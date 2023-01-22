/*!
  \file		textured_quad_display.h
  \author	Toshio Ueshiba
*/
#pragma once

#include <QObject>

#ifndef Q_MOC_RUN
#  include <mutex>
#  include <sensor_msgs/Image.h>
#  include <OGRE/OgreEntity.h>
#  include <OGRE/OgreManualObject.h>
#  include <OGRE/OgreRenderQueueListener.h>
#  include <OGRE/OgreRenderSystem.h>
#  include <OGRE/OgreRenderTargetListener.h>
#  include <OGRE/OgreRenderWindow.h>
#  include <OGRE/OgreRoot.h>
#  include <OGRE/OgreSceneNode.h>
#  include <OGRE/OgreVector3.h>
#  include <OGRE/OgreWindowEventUtilities.h>
#  include <rviz/display.h>
#  include <rviz/frame_manager.h>
#  include <rviz/image/image_display_base.h>
#  include <rviz/image/ros_image_texture.h>
#  include <aist_lenti_mark/QuadStamped.h>
#endif  // Q_MOC_RUN

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class FloatProperty;
class RenderPanel;
class RosTopicProperty;
class TfFrameProperty;

/************************************************************************
*  class TexturedQuadDisplay						*
************************************************************************/
class TexturedQuadDisplay: public rviz::Display,
			   public Ogre::RenderTargetListener,
			   public Ogre::RenderQueueListener
{
    Q_OBJECT
  public:
			TexturedQuadDisplay()				;
    virtual		~TexturedQuadDisplay()				;

  // Overrides from Display
    virtual void	onInitialize()					;
    virtual void	onEnable()					;
    virtual void	onDisable()					;
    virtual void	update(float wall_dt, float ros_dt)		;
    virtual void	reset()						;

  protected:
    virtual void	subscribe()					;
    virtual void	unsubscribe()					;

  private:
    using image_cp = sensor_msgs::Image::ConstPtr;
    using quad_cp  = aist_lenti_mark::QuadStampedConstPtr;

    void		updateImage(const image_cp& image)		;
    void		updateQuad(const quad_cp& image)		;

    void		createTexture()					;
    void		createMesh()					;
    void		updateMeshProperties()				;
    void		updateCamera()					;

  private Q_SLOTS:
    void		updateDisplayImages()				;

  private:
    std::unique_ptr<RosTopicProperty>	image_topic_property_;
    std::unique_ptr<RosTopicProperty>	quad_topic_property_;

    ros::NodeHandle			nh_;
    ros::Subscriber			image_sub_;
    ros::Subscriber			quad_sub_;

    image_cp				cur_image_;
    quad_cp				cur_quad_;

    std::unique_ptr<Ogre::SceneNode>	mesh_node_;
    std::unique_ptr<Ogre::ManualObject>	manual_object_;
    Ogre::MaterialPtr			mesh_material_;
    std::unique_ptr<ROSImageTexture>	texture_;

    std::mutex				image_mutex_;
    std::mutex				quad_mutex_;
};

}  // namespace rviz
