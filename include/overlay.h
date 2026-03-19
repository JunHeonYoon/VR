//====== Copyright Valve Corporation, All rights reserved. =======

#ifndef OPENVROVERLAYCONTROLLER_H
#define OPENVROVERLAYCONTROLLER_H

#ifdef _WIN32
#pragma once
#endif

#include "openvr.h"

#include <QtCore/QtCore>
// because of incompatibilities with QtOpenGL and GLEW we need to cherry pick includes
#include <QtGui/QVector2D>
#include <QtGui/QMatrix4x4>
#include <QtCore/QVector>
#include <QtGui/QVector2D>
#include <QtGui/QVector3D>
#include <QtGui/QOpenGLContext>
#include <QtGui/QOpenGLFramebufferObject>
#include <QtWidgets/QGraphicsScene>
#include <QtGui/QOffscreenSurface>

#include <iostream>
#include <stdio.h>

#include <QtWidgets/QWidget>
#include <QMainWindow>
#include <QTimer>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// --------------------
// ROS2 (Humble) includes
// --------------------
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>


namespace Ui {
class OverlayWidget;
}

class OverlayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit OverlayWidget(QWidget *parent = 0);
    ~OverlayWidget();

    void WInit();

    // ROS2: sensor_msgs::ImageConstPtr -> sensor_msgs::msg::Image::ConstSharedPtr
    void update_rviz(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    // ROS2: std_msgs::String::ConstPtr -> std_msgs::msg::String::ConstSharedPtr
    void commandCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    void update_status(const std_msgs::msg::String::ConstSharedPtr msg);

    // ROS2: std_msgs::Bool::ConstPtr -> std_msgs::msg::Bool::ConstSharedPtr
    void tracker_status(const std_msgs::msg::Bool::ConstSharedPtr msg);

    //private slots:

private:

    int argc;
    char **argv;
    Ui::OverlayWidget *ui;
    static const uint32_t trackerNum = 4;

    QImage qt_image;
};


class OverlayController : public QObject
{
    Q_OBJECT
    typedef QObject BaseClass;

public:
    static OverlayController *SharedInstance();

public:
    OverlayController();
    virtual ~OverlayController();

    bool Init();
    void Shutdown();
    void EnableRestart();

    bool BHMDAvailable();
    vr::IVRSystem *GetVRSystem();
    vr::HmdError GetLastHmdError();

    QString GetVRDriverString();
    QString GetVRDisplayString();
    QString GetName() { return m_strName; }

    void SetWidget( QWidget *pWidget );
    void OnSceneUpdate();

    void ShowRviz();
    void HideRviz();
    void MoveOverlayRight();
    void MoveOverlayLeft();
    void MoveOverlayUp();
    void MoveOverlayDown();
    void MoveOverlayFront();
    void MoveOverlayBack();
    void ChangeOpacity(float number);

    int loop_tick = 0;

public slots:
    void OnSceneChanged( const QList<QRectF>& );

protected:

private:
    bool ConnectToVRRuntime();
    void DisconnectFromVRRuntime();

    vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
    QString m_strVRDriver;
    QString m_strVRDisplay;
    QString m_strName;

    vr::HmdError m_eLastHmdError;

private:

    vr::HmdError m_eCompositorError;
    vr::HmdError m_eOverlayError;
    vr::VROverlayHandle_t m_ulOverlayHandle;
    vr::VROverlayHandle_t m_ulOverlayThumbnailHandle;

    QOpenGLContext *m_pOpenGLContext;
    QGraphicsScene *m_pScene;
    QOpenGLFramebufferObject *m_pFbo;
    QOffscreenSurface *m_pOffscreenSurface;

    // the widget we're drawing into the texture
    QWidget *m_pWidget;
};

#endif // OPENVROVERLAYCONTROLLER_H
