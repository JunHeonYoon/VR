//====== Copyright Valve Corporation, All rights reserved. =======

#include "overlay.h"

#include <QOpenGLFramebufferObjectFormat>
#include <QOpenGLPaintDevice>
#include <QPainter>
#include <QtWidgets/QWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QCursor>

#include "ui_widget.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace vr;

// ----------------------------------------------------------------------------
// OpenVR overlay transform (position)
// ----------------------------------------------------------------------------
vr::HmdMatrix34_t transform = {
    1.0f, 0.0f, 0.0f, 0.6f,   // +x : right
    0.0f, 1.0f, 0.0f, 0.3f,   // +y : up
    0.0f, 0.0f, 1.0f, -1.0f   // -z : forward
};

// ----------------------------------------------------------------------------
// ROS2 context storage (no header changes)
// ----------------------------------------------------------------------------
namespace
{
struct OverlayRosContext
{
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  std::thread spin_thread;
  std::atomic<bool> running{false};

  image_transport::Subscriber image_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracker_sub;
};

std::mutex g_ctx_mtx;
std::unordered_map<OverlayWidget*, OverlayRosContext> g_ctx;

inline OverlayRosContext* get_ctx(OverlayWidget* self)
{
  std::lock_guard<std::mutex> lk(g_ctx_mtx);
  auto it = g_ctx.find(self);
  if (it == g_ctx.end()) return nullptr;
  return &it->second;
}

inline OverlayRosContext& ensure_ctx(OverlayWidget* self)
{
  std::lock_guard<std::mutex> lk(g_ctx_mtx);
  return g_ctx[self];
}

inline void erase_ctx(OverlayWidget* self)
{
  std::lock_guard<std::mutex> lk(g_ctx_mtx);
  g_ctx.erase(self);
}
} // namespace

// ----------------------------------------------------------------------------
// OverlayWidget
// ----------------------------------------------------------------------------
OverlayWidget::OverlayWidget(QWidget *parent)
  : QWidget(parent),
    ui(new Ui::OverlayWidget)
{
  ui->setupUi(this);
  std::cout << "UI setup" << std::endl;
}

OverlayWidget::~OverlayWidget()
{
  // Stop ROS spinning thread safely (if started)
  auto* ctx = get_ctx(this);
  if (ctx)
  {
    ctx->running.store(false);

    if (ctx->exec)
    {
      try {
        ctx->exec->cancel();
      } catch (...) {
        // ignore
      }
    }

    if (ctx->spin_thread.joinable())
      ctx->spin_thread.join();

    // Make sure node is removed before destruction
    if (ctx->exec && ctx->node)
    {
      try {
        ctx->exec->remove_node(ctx->node);
      } catch (...) {
        // ignore
      }
    }

    erase_ctx(this);
  }

  delete ui;
}

void OverlayWidget::WInit()
{
  std::cout << "overlay image subscribing..." << std::endl;

  // NOTE: rclcpp::init(...) should be called in main.
  // Avoid calling init here to prevent double-init.

  auto& ctx = ensure_ctx(this);

  // If already initialized, do nothing
  if (ctx.node)
    return;

  ctx.node = std::make_shared<rclcpp::Node>("overlay_widget");
  ctx.exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ctx.exec->add_node(ctx.node);

  // ROS2 image_transport subscription (recommended API)
  ctx.image_sub = image_transport::create_subscription(
      ctx.node.get(),
      "/rviz1/camera1/image",
      std::bind(&OverlayWidget::update_rviz, this, std::placeholders::_1),
      "raw");

  // Command / status / tracker subscriptions
  ctx.command_sub = ctx.node->create_subscription<std_msgs::msg::String>(
      "overlay_command",
      rclcpp::QoS(10),
      std::bind(&OverlayWidget::commandCallback, this, std::placeholders::_1));

  ctx.status_sub = ctx.node->create_subscription<std_msgs::msg::String>(
      "/tocabi_status",
      rclcpp::QoS(1),
      std::bind(&OverlayWidget::update_status, this, std::placeholders::_1));

  ctx.tracker_sub = ctx.node->create_subscription<std_msgs::msg::Bool>(
      "TRACKERSTATUS",
      rclcpp::QoS(1),
      std::bind(&OverlayWidget::tracker_status, this, std::placeholders::_1));

//   cv::namedWindow("rviz_image");

  // Spin in background thread (Qt event loop must stay unblocked)
  ctx.running.store(true);
  ctx.spin_thread = std::thread([this]()
  {
    auto* c = get_ctx(this);
    if (!c || !c->exec) return;

    while (c->running.load())
    {
      // spin_some keeps it responsive and avoids blocking forever
      c->exec->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
}

void OverlayWidget::update_status(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) return;

  ui->label_2->clear();
  ui->label_2->setText(msg->data.c_str());
  ui->label_2->show();

  ui->textEdit->append(msg->data.c_str());
  ui->textEdit->setStyleSheet("QTextEdit { color : black; }");
}

void OverlayWidget::commandCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg) return;

  const std::string cmd = msg->data;

  if (cmd == "close")
  {
    std::cout << "close overlay" << std::endl;
    OverlayController::SharedInstance()->HideRviz();
  }
  else if (cmd == "open")
  {
    std::cout << "open overlay" << std::endl;
    OverlayController::SharedInstance()->ShowRviz();
  }
  else if (cmd == "right")
  {
    std::cout << "move overlay right" << std::endl;
    OverlayController::SharedInstance()->MoveOverlayRight();
  }
  else if (cmd == "left")
  {
    std::cout << "move overlay left" << std::endl;
    OverlayController::SharedInstance()->MoveOverlayLeft();
  }
  else if (cmd == "up")
  {
    std::cout << "move overlay up" << std::endl;
    OverlayController::SharedInstance()->MoveOverlayUp();
  }
  else if (cmd == "down")
  {
    std::cout << "move overlay down" << std::endl;
    OverlayController::SharedInstance()->MoveOverlayDown();
  }
  else if (cmd == "front")
  {
    std::cout << "move overlay front" << std::endl;
    OverlayController::SharedInstance()->MoveOverlayFront();
  }
  else if (cmd == "back")
  {
    std::cout << "move overlay back" << std::endl;
    OverlayController::SharedInstance()->MoveOverlayBack();
  }
  else
  {
    // Unknown command -> ignore (or print)
    // std::cout << "unknown overlay_command: " << cmd << std::endl;
  }
}

void OverlayWidget::tracker_status(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (!msg) return;

  if (!msg->data)
  {
    const char* txt = "TRACKERS DISCONNECTED";
    ui->label_3->clear();
    ui->label_3->setText(txt);
    ui->label_3->setStyleSheet("QLabel { background-color : rgba(169,169,169,0%); color : red; }");
    ui->label_3->show();
  }
  else
  {
    const char* txt = "TRACKERS CONNECTED";
    ui->label_3->clear();
    ui->label_3->setText(txt);
    ui->label_3->setStyleSheet("QLabel { background-color : rgba(169,169,169,0%); color : red; }");
    ui->label_3->show();
  }
}

void OverlayWidget::update_rviz(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!msg) return;

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("overlay_widget"), "cv_bridge exception: %s", e.what());
    return;
  }

//   cv::imshow("rviz_image", cv_ptr->image);
//   cv::waitKey(3);

  qt_image = QImage(
      reinterpret_cast<const unsigned char*>(cv_ptr->image.data),
      cv_ptr->image.cols,
      cv_ptr->image.rows,
      QImage::Format_RGB888);

  ui->label->setPixmap(QPixmap::fromImage(qt_image));
  if (ui->label->pixmap())
    ui->label->resize(ui->label->pixmap()->size());
}

// ----------------------------------------------------------------------------
// OverlayController
// ----------------------------------------------------------------------------
OverlayController *s_pSharedVRController = NULL;

OverlayController *OverlayController::SharedInstance()
{
  if (!s_pSharedVRController)
    s_pSharedVRController = new OverlayController();
  return s_pSharedVRController;
}

OverlayController::OverlayController()
  : BaseClass()
  , m_strVRDriver("No Driver")
  , m_strVRDisplay("No Display")
  , m_eLastHmdError(vr::VRInitError_None)
  , m_eCompositorError(vr::VRInitError_None)
  , m_eOverlayError(vr::VRInitError_None)
  , m_ulOverlayHandle(vr::k_ulOverlayHandleInvalid)
  , m_pOpenGLContext(NULL)
  , m_pScene(NULL)
  , m_pFbo(NULL)
  , m_pOffscreenSurface(NULL)
  , m_pWidget(NULL)
{
}

OverlayController::~OverlayController() {}

static QString GetTrackedDeviceString(vr::IVRSystem *pHmd,
                                      vr::TrackedDeviceIndex_t unDevice,
                                      vr::TrackedDeviceProperty prop)
{
  char buf[128];
  vr::TrackedPropertyError err;
  pHmd->GetStringTrackedDeviceProperty(unDevice, prop, buf, sizeof(buf), &err);
  if (err != vr::TrackedProp_Success)
    return QString("Error Getting String: ") + pHmd->GetPropErrorNameFromEnum(err);
  return buf;
}

bool OverlayController::Init()
{
  bool bSuccess = true;
  m_strName = "systemoverlay";

  QStringList arguments = qApp->arguments();
  int nNameArg = arguments.indexOf("-name");
  if (nNameArg != -1 && nNameArg + 2 <= arguments.size())
    m_strName = arguments.at(nNameArg + 1);

  QSurfaceFormat format;
  format.setMajorVersion(4);
  format.setMinorVersion(1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);

  m_pOpenGLContext = new QOpenGLContext();
  m_pOpenGLContext->setFormat(format);
  bSuccess = m_pOpenGLContext->create();
  if (!bSuccess)
    return false;

  m_pOffscreenSurface = new QOffscreenSurface();
  m_pOffscreenSurface->create();
  m_pOpenGLContext->makeCurrent(m_pOffscreenSurface);

  m_pScene = new QGraphicsScene();
  connect(m_pScene, SIGNAL(changed(const QList<QRectF>&)),
          this, SLOT(OnSceneChanged(const QList<QRectF>&)));

  bSuccess = ConnectToVRRuntime();
  bSuccess = bSuccess && (vr::VRCompositor() != NULL);

  if (vr::VROverlay())
  {
    std::string sKey = std::string("sample.") + m_strName.toStdString();
    vr::VROverlayError overlayError =
      vr::VROverlay()->CreateOverlay(sKey.c_str(),
                                     m_strName.toStdString().c_str(),
                                     &m_ulOverlayHandle);
    bSuccess = bSuccess && (overlayError == vr::VROverlayError_None);
  }

  if (bSuccess)
  {
    vr::VROverlay()->SetOverlayWidthInMeters(m_ulOverlayHandle, 0.4f);
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(
        m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
    vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle, 0.8f);
    vr::VROverlay()->ShowOverlay(m_ulOverlayHandle);
  }

  std::cout << "Overlay setup complete" << std::endl;
  return bSuccess;
}

void OverlayController::Shutdown()
{
  DisconnectFromVRRuntime();

  delete m_pScene; m_pScene = nullptr;
  delete m_pFbo; m_pFbo = nullptr;
  delete m_pOffscreenSurface; m_pOffscreenSurface = nullptr;

  if (m_pOpenGLContext)
  {
    delete m_pOpenGLContext;
    m_pOpenGLContext = NULL;
  }
}

void OverlayController::OnSceneChanged(const QList<QRectF>&)
{
  if ((m_ulOverlayHandle == k_ulOverlayHandleInvalid) || !vr::VROverlay() ||
      (!vr::VROverlay()->IsOverlayVisible(m_ulOverlayHandle)))
    return;

  if (!m_pFbo || !m_pOpenGLContext || !m_pOffscreenSurface || !m_pScene)
    return;

  m_pOpenGLContext->makeCurrent(m_pOffscreenSurface);
  m_pFbo->bind();

  QOpenGLPaintDevice device(m_pFbo->size());
  QPainter painter(&device);
  m_pScene->render(&painter);

  m_pFbo->release();

  GLuint unTexture = m_pFbo->texture();
  if (unTexture != 0)
  {
    vr::Texture_t texture = { (void*)(uintptr_t)unTexture,
                              vr::TextureType_OpenGL,
                              vr::ColorSpace_Auto };
    vr::VROverlay()->SetOverlayTexture(m_ulOverlayHandle, &texture);
  }

  if (loop_tick % 100 == 0)
    std::cout << "scene update" << std::endl;
  loop_tick++;
}

void OverlayController::OnSceneUpdate()
{
  // same as OnSceneChanged but callable manually
  if ((m_ulOverlayHandle == k_ulOverlayHandleInvalid) || !vr::VROverlay() ||
      (!vr::VROverlay()->IsOverlayVisible(m_ulOverlayHandle)))
    return;

  if (!m_pFbo || !m_pOpenGLContext || !m_pOffscreenSurface || !m_pScene)
    return;

  m_pOpenGLContext->makeCurrent(m_pOffscreenSurface);
  m_pFbo->bind();

  QOpenGLPaintDevice device(m_pFbo->size());
  QPainter painter(&device);
  m_pScene->render(&painter);

  m_pFbo->release();

  GLuint unTexture = m_pFbo->texture();
  if (unTexture != 0)
  {
    vr::Texture_t texture = { (void*)(uintptr_t)unTexture,
                              vr::TextureType_OpenGL,
                              vr::ColorSpace_Auto };
    vr::VROverlay()->SetOverlayTexture(m_ulOverlayHandle, &texture);
  }
}

void OverlayController::SetWidget(QWidget *pWidget)
{
  if (m_pScene)
  {
    pWidget->move(0, 0);
    m_pScene->addWidget(pWidget);
  }
  m_pWidget = pWidget;

  // Recreate FBO for widget size
  delete m_pFbo;
  m_pFbo = new QOpenGLFramebufferObject(pWidget->width(), pWidget->height(), GL_TEXTURE_2D);

  if (vr::VROverlay())
  {
    vr::HmdVector2_t vecWindowSize = {
      (float)pWidget->width(),
      (float)pWidget->height()
    };
    vr::VROverlay()->SetOverlayMouseScale(m_ulOverlayHandle, &vecWindowSize);
  }

  std::cout << "SetWidget complete" << std::endl;
}

bool OverlayController::ConnectToVRRuntime()
{
  m_eLastHmdError = vr::VRInitError_None;
  vr::IVRSystem *pVRSystem = vr::VR_Init(&m_eLastHmdError, vr::VRApplication_Overlay);

  if (m_eLastHmdError != vr::VRInitError_None)
  {
    m_strVRDriver = "No Driver";
    m_strVRDisplay = "No Display";
    return false;
  }

  m_strVRDriver  = GetTrackedDeviceString(pVRSystem, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
  m_strVRDisplay = GetTrackedDeviceString(pVRSystem, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
  return true;
}

void OverlayController::ShowRviz()
{
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle, 0.5f);
}

void OverlayController::HideRviz()
{
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle, 0.0f);
}

void OverlayController::MoveOverlayRight()
{
  transform.m[0][3] += 0.01f;
  std::cout << transform.m[0][3] << std::endl;
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

void OverlayController::MoveOverlayLeft()
{
  transform.m[0][3] -= 0.01f;
  std::cout << transform.m[0][3] << std::endl;
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

void OverlayController::MoveOverlayUp()
{
  transform.m[1][3] += 0.01f;
  std::cout << transform.m[1][3] << std::endl;
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

void OverlayController::MoveOverlayDown()
{
  transform.m[1][3] -= 0.01f;
  std::cout << transform.m[1][3] << std::endl;
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

void OverlayController::MoveOverlayFront()
{
  transform.m[2][3] += 0.01f;
  std::cout << transform.m[2][3] << std::endl;
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

void OverlayController::MoveOverlayBack()
{
  transform.m[2][3] -= 0.01f;
  std::cout << transform.m[2][3] << std::endl;
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

void OverlayController::ChangeOpacity(float number)
{
  if (vr::VROverlay())
    vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle, number);
}

void OverlayController::DisconnectFromVRRuntime()
{
  vr::VR_Shutdown();
}

QString OverlayController::GetVRDriverString()
{
  return m_strVRDriver;
}

QString OverlayController::GetVRDisplayString()
{
  return m_strVRDisplay;
}

bool OverlayController::BHMDAvailable()
{
  return vr::VRSystem() != NULL;
}

vr::HmdError OverlayController::GetLastHmdError()
{
  return m_eLastHmdError;
}
