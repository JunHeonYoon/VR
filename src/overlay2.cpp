//====== Copyright Valve Corporation, All rights reserved. =======


#include "overlay.h"


#include <QOpenGLFramebufferObjectFormat>
#include <QOpenGLPaintDevice>
#include <QPainter>
#include <QtWidgets/QWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QCursor>

#include "ui_widget2.h"

#include <vector>

using namespace vr;

vr::HmdMatrix34_t transform = {								//overlay position
	1.0f, 0.0f, 0.0f, 0.0f,									//+: right horizontal axis
	0.0f, 1.0f, 0.0f, 0.0f,									//+: upper axis
	0.0f, 0.0f, 1.0f, -1.0f									//-: forward axis
};

OverlayWidget::OverlayWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::OverlayWidget)
{
    ui->setupUi(this);
    
	std::cout << "UI setup" << std::endl;
}

OverlayWidget::~OverlayWidget()
{
    delete ui;
}

void OverlayWidget::WInit()
{
    std::cout << "overlay image subscribing..." << std::endl;

    // (권장) main에서 rclcpp::init(argc, argv) 호출해두는 방식.
    // 여기서 직접 init하려면 중복 호출에 주의.
    auto node = std::make_shared<rclcpp::Node>("overlay_widget");

    // ---- (원래 주석처리 돼 있던 것들: 그대로 ROS2 버전으로 유지 가능) ----
    // image_transport::ImageTransport it(node);
    //
    // auto sub = it.subscribe(
    //     "/rviz1/camera1/image", 1,
    //     std::bind(&OverlayWidget::update_rviz, this, std::placeholders::_1)
    // );
    //
    // auto command_sub = node->create_subscription<std_msgs::msg::String>(
    //     "overlay_command", 10,
    //     std::bind(&OverlayWidget::commandCallback, this, std::placeholders::_1)
    // );
    //
    // auto status_sub = node->create_subscription<std_msgs::msg::String>(
    //     "/tocabi_status", 1,
    //     std::bind(&OverlayWidget::update_status, this, std::placeholders::_1)
    // );

    auto trackers_sub = node->create_subscription<std_msgs::msg::Bool>(
        "TRACKERSTATUS", rclcpp::QoS(1),
        std::bind(&OverlayWidget::tracker_status, this, std::placeholders::_1)
    );

    // ROS2 spin
    rclcpp::spin(node);
}

void OverlayWidget::update_status(const std_msgs::msg::String::ConstSharedPtr msg)
{
    ui->label_2->clear();
    ui->label_2->setText(msg->data.c_str());
    ui->label_2->show();

    ui->textEdit->append(msg->data.c_str());
    ui->textEdit->setStyleSheet("QTextEdit { color : black; }");
}


void OverlayWidget::commandCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
    std::string opacity = msg->data;
    std::istringstream ss(opacity);
    std::string stringBuffer;
    std::vector<std::string> xx;
    xx.clear();
    while(getline(ss, stringBuffer, ' '))
    {
        xx.push_back(stringBuffer);
    }

    if(0 == strcmp(msg->data.c_str(), "close"))
    {
        std::cout << "close overlay" << std::endl;
        OverlayController::SharedInstance()->HideRviz();
    }
    else if(0 == strcmp(msg->data.c_str(), "open"))
    {
        std::cout << "open overlay" << std::endl;
        OverlayController::SharedInstance()->ShowRviz();
    }
    else if(0 == strcmp(msg->data.c_str(), "right"))
    {
        std::cout << "move overlay right" << std::endl;
        OverlayController::SharedInstance()->MoveOverlayRight();
    }
    else if(0 == strcmp(msg->data.c_str(), "left"))
    {
        std::cout << "move overlay left" << std::endl;
        OverlayController::SharedInstance()->MoveOverlayLeft();
    }
    else if(0 == strcmp(msg->data.c_str(), "up"))
    {
        std::cout << "move overlay up" << std::endl;
        OverlayController::SharedInstance()->MoveOverlayUp();
    }
    else if(0 == strcmp(msg->data.c_str(), "down"))
    {
        std::cout << "move overlay down" << std::endl;
        OverlayController::SharedInstance()->MoveOverlayDown();
    }
    else if(0 == strcmp(msg->data.c_str(), "front"))
    {
        std::cout << "move overlay front" << std::endl;
        OverlayController::SharedInstance()->MoveOverlayFront();
    }
    else if(0 == strcmp(msg->data.c_str(), "back"))
    {
        std::cout << "move overlay back" << std::endl;
        OverlayController::SharedInstance()->MoveOverlayBack();
    }
}


void OverlayWidget::tracker_status(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
    if (!msg->data)
    {
        std::string tracker_flag1 = "TRACKERS DISCONNECTED";
        ui->label_3->clear();
        ui->label_3->setText(tracker_flag1.c_str());
        ui->label_3->setStyleSheet("QLabel { background-color : rgba(169,169,169,0%); color : red; }");
        ui->label_3->show();

        OverlayController::SharedInstance()->ShowRviz();
    }
    else
    {
        std::string tracker_flag = "TRACKERS CONNECTED";
        ui->label_3->clear();
        ui->label_3->setText(tracker_flag.c_str());
        ui->label_3->setStyleSheet("QLabel { background-color : rgba(169,169,169,0%); color : red; }");
        ui->label_3->show();

        OverlayController::SharedInstance()->HideRviz();
    }
}

void OverlayWidget::update_rviz(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    // cv_bridge::CvImagePtr cv_ptr;
    // try
    // {
    //   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	// }
    // catch (cv_bridge::Exception& e)
    // {
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    //   return;
    // }

    // Update GUI Window
    // cv::imshow("rviz_image", cv_ptr->image);
    // cv::waitKey(3);

    // qt_image = QImage((const unsigned char*) (cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_RGB888);

	// std::cout << cv_ptr->image.cols << std::endl;
	// std::cout << cv_ptr->image.rows << std::endl;

    // ui->label->setPixmap(QPixmap::fromImage(qt_image));
	// ui->label->pixmap();
    // ui->label->resize(ui->label->pixmap()->size());
	
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
OverlayController *s_pSharedVRController = NULL;

OverlayController *OverlayController::SharedInstance()
{
	if ( !s_pSharedVRController )
	{
        s_pSharedVRController = new OverlayController();
	}
	return s_pSharedVRController;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
OverlayController::OverlayController()
	: BaseClass()
	, m_strVRDriver( "No Driver" )
	, m_strVRDisplay( "No Display" )
    , m_eLastHmdError( vr::VRInitError_None )
    , m_eCompositorError( vr::VRInitError_None )
    , m_eOverlayError( vr::VRInitError_None )
	, m_ulOverlayHandle( vr::k_ulOverlayHandleInvalid )
	, m_pOpenGLContext( NULL )
	, m_pScene( NULL )
	, m_pFbo( NULL )
	, m_pOffscreenSurface ( NULL )
	, m_pWidget( NULL )
{
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
OverlayController::~OverlayController()
{
}


//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a QString
//-----------------------------------------------------------------------------
QString GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop )
{
	char buf[128];
	vr::TrackedPropertyError err;
	pHmd->GetStringTrackedDeviceProperty( unDevice, prop, buf, sizeof( buf ), &err );
	if( err != vr::TrackedProp_Success )
	{
		return QString( "Error Getting String: " ) + pHmd->GetPropErrorNameFromEnum( err );
	}
	else
	{
		return buf;
	}
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool OverlayController::Init()
{
	bool bSuccess = true;

    m_strName = "systemoverlay";

	QStringList arguments = qApp->arguments();

	int nNameArg = arguments.indexOf( "-name" );
	if( nNameArg != -1 && nNameArg + 2 <= arguments.size() )
	{
		m_strName = arguments.at( nNameArg + 1 );
	}

	QSurfaceFormat format;

	//set opengl version (min,max)
	format.setMajorVersion( 4 );
	format.setMinorVersion( 1 );
	format.setProfile( QSurfaceFormat::CompatibilityProfile );


	//Sets the format the OpenGL context
	m_pOpenGLContext = new QOpenGLContext();
	m_pOpenGLContext->setFormat( format );
	bSuccess = m_pOpenGLContext->create();
	if( !bSuccess )
		return false;

	// create an offscreen surface to attach the context and FBO to
	m_pOffscreenSurface = new QOffscreenSurface();
	m_pOffscreenSurface->create();
	m_pOpenGLContext->makeCurrent( m_pOffscreenSurface );

	
	m_pScene = new QGraphicsScene();
	connect( m_pScene, SIGNAL(changed(const QList<QRectF>&)), this, SLOT( OnSceneChanged(const QList<QRectF>&)) );

	// Loading the OpenVR Runtime
	bSuccess = ConnectToVRRuntime();

    bSuccess = bSuccess && vr::VRCompositor() != NULL;

    if( vr::VROverlay() )
	{
        std::string sKey = std::string( "sample." ) + m_strName.toStdString();
        vr::VROverlayError overlayError = vr::VROverlay()->CreateOverlay( sKey.c_str(), m_strName.toStdString().c_str(), &m_ulOverlayHandle);
		bSuccess = bSuccess && overlayError == vr::VROverlayError_None;
	}

	if( bSuccess )
	{
		vr::VROverlay()->SetOverlayWidthInMeters( m_ulOverlayHandle, 2.0f );
        vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
		vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle,0.0);
		vr::VROverlay()->ShowOverlay(m_ulOverlayHandle);
	}
	std::cout << "Overlay setup complete" << std::endl;
	return true;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::Shutdown()
{
	DisconnectFromVRRuntime();

	delete m_pScene;
	delete m_pFbo;
	delete m_pOffscreenSurface;

	if( m_pOpenGLContext )
	{
//		m_pOpenGLContext->destroy();
		delete m_pOpenGLContext;
		m_pOpenGLContext = NULL;
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::OnSceneChanged( const QList<QRectF>& )
{
	// skip rendering if the overlay isn't visible
    if( ( m_ulOverlayHandle == k_ulOverlayHandleInvalid ) || !vr::VROverlay() ||
        ( !vr::VROverlay()->IsOverlayVisible( m_ulOverlayHandle ) ))
        return;

	m_pOpenGLContext->makeCurrent( m_pOffscreenSurface );
	m_pFbo->bind();
	
	QOpenGLPaintDevice device( m_pFbo->size() );
	QPainter painter( &device );

	m_pScene->render( &painter );

	m_pFbo->release();

	GLuint unTexture = m_pFbo->texture();
	if( unTexture != 0 )
	{
        vr::Texture_t texture = {(void*)(uintptr_t)unTexture, vr::TextureType_OpenGL, vr::ColorSpace_Auto };
        vr::VROverlay()->SetOverlayTexture( m_ulOverlayHandle, &texture );
	}
	if (loop_tick % 100 == 0)
		std::cout << "scene update" << std::endl;
	loop_tick++;
}

//-----------------------------------------------------------------------------
// Purpose: update qt5 image
//-----------------------------------------------------------------------------

void OverlayController::OnSceneUpdate()
{
	// skip rendering if the overlay isn't visible
    if( ( m_ulOverlayHandle == k_ulOverlayHandleInvalid ) || !vr::VROverlay() ||
        ( !vr::VROverlay()->IsOverlayVisible( m_ulOverlayHandle ) ))
        return;

	m_pOpenGLContext->makeCurrent( m_pOffscreenSurface );
	m_pFbo->bind();
		
	QOpenGLPaintDevice device( m_pFbo->size() );
	QPainter painter( &device );

	m_pScene->render( &painter );

	m_pFbo->release();

	GLuint unTexture = m_pFbo->texture();
	if( unTexture != 0 )
	{
        vr::Texture_t texture = {(void*)(uintptr_t)unTexture, vr::TextureType_OpenGL, vr::ColorSpace_Auto };
        vr::VROverlay()->SetOverlayTexture( m_ulOverlayHandle, &texture );
	}
	std::cout << "888" << std::endl;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::SetWidget( QWidget *pWidget )
{
	if( m_pScene )
	{
		// all of the mouse handling stuff requires that the widget be at 0,0
		pWidget->move( 0, 0 );
		m_pScene->addWidget( pWidget );
	}
	m_pWidget = pWidget;

	m_pFbo = new QOpenGLFramebufferObject( pWidget->width(), pWidget->height(), GL_TEXTURE_2D );

    if( vr::VROverlay() )
    {
        vr::HmdVector2_t vecWindowSize =
        {
            (float)pWidget->width(),
            (float)pWidget->height()
        };
        vr::VROverlay()->SetOverlayMouseScale( m_ulOverlayHandle, &vecWindowSize );
    }
	std::cout << "66" << std::endl;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool OverlayController::ConnectToVRRuntime()
{
    m_eLastHmdError = vr::VRInitError_None;
    vr::IVRSystem *pVRSystem = vr::VR_Init( &m_eLastHmdError, vr::VRApplication_Overlay );

    if ( m_eLastHmdError != vr::VRInitError_None )
	{
		m_strVRDriver = "No Driver";
		m_strVRDisplay = "No Display";
		return false;
	}

    m_strVRDriver = GetTrackedDeviceString(pVRSystem, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
    m_strVRDisplay = GetTrackedDeviceString(pVRSystem, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);

	return true;
}

//-----------------------------------------------------------------------------
// Purpose: overlay on function
//-----------------------------------------------------------------------------
void OverlayController::ShowRviz()
{
	vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle,0.5);
}

//-----------------------------------------------------------------------------
// Purpose: overlay off function
//-----------------------------------------------------------------------------
void OverlayController::HideRviz()
{
	vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle,0);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::MoveOverlayRight()
{
	transform.m[0][3]= transform.m[0][3] + 0.01;
	std::cout << transform.m[0][3] << std::endl;
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::MoveOverlayLeft()
{
	transform.m[0][3]= transform.m[0][3] - 0.01;
	std::cout << transform.m[0][3] << std::endl;
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::MoveOverlayUp()
{
	transform.m[1][3]= transform.m[1][3] + 0.01;
	std::cout << transform.m[1][3] << std::endl;
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::MoveOverlayDown()
{
	transform.m[1][3]= transform.m[1][3] - 0.01;
	std::cout << transform.m[1][3] << std::endl;
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::MoveOverlayFront()
{
	transform.m[2][3]= transform.m[2][3] + 0.01;
	std::cout << transform.m[2][3] << std::endl;
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::MoveOverlayBack()
{
	transform.m[2][3]= transform.m[2][3] - 0.01;
	std::cout << transform.m[2][3] << std::endl;
	vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &transform);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::ChangeOpacity(float number)
{
	vr::VROverlay()->SetOverlayAlpha(m_ulOverlayHandle, number);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void OverlayController::DisconnectFromVRRuntime()
{
	vr::VR_Shutdown();
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
QString OverlayController::GetVRDriverString()
{
	return m_strVRDriver;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
QString OverlayController::GetVRDisplayString()
{
	return m_strVRDisplay;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool OverlayController::BHMDAvailable()
{
    return vr::VRSystem() != NULL;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

vr::HmdError OverlayController::GetLastHmdError()
{
	return m_eLastHmdError;
}


