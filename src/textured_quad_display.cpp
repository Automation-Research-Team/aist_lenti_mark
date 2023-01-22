/*!
  \file		textured_quad_display.cpp
  \author	Toshio Ueshiba
*/
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <cv_bridge/cv_bridge.h>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/validate_floats.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <sensor_msgs/image_encodings.h>
#include "textured_quad_display.h"

namespace rviz
{
static Ogre::Vector3
fromMsg(const geometry_msgs::Point& p)
{
    return {p.x, p.y, p.z};
}

/************************************************************************
*  class TexturedQuadDisplay						*
************************************************************************/
TexturedQuadDisplay::TexturedQuadDisplay()
    :Display(),
     image_topic_property_(new RosTopicProperty(
			       "Image Topic", "",
			       QString::fromStdString(
				   ros::message_traits::datatype<
				       sensor_msgs::Image>()),
			       "Image topic to subscribe to.",
			       this, SLOT(updateDisplayImages()))),
     quad_topic_property_(new RosTopicProperty(
			      "Quad Topic", "",
			      QString::fromStdString(
				  ros::message_traits::datatype<
				      aist_lenti_mark::QuadStamped>()),
			      "Quad topic to subscribe to.",
			      this, SLOT(updateDisplayImages())))
{
}

TexturedQuadDisplay::~TexturedQuadDisplay()
{
    unsubscribe();
}

/*
 *  public mamber functions : Overrides from Display
 */
void
TexturedQuadDisplay::onInitialize()
{
    Display::onInitialize();
}

void
TexturedQuadDisplay::onEnable()
{
    subscribe();
}

void
TexturedQuadDisplay::onDisable()
{
    unsubscribe();
}

void
TexturedQuadDisplay::update(float wall_dt, float ros_dt)
{
    try
    {
	if (cur_quad_ && cur_image_)
	{
	    createTexture();	// Create texture_ from cur_image_
	    createMesh();
	    updateMeshProperties();

	    if (texture_ &&
		!image_topic_property_->getTopic().isEmpty() &&
		!quad_topic_property_->getTopic().isEmpty())
	    {
		texture_->update();
		updateCamera();
	    }
	}
    }
    catch (const std::exception& e)
    {
	setStatus(StatusProperty::Error, "Display Image", e.what());
	return;
    }

    setStatus(StatusProperty::Ok, "Display Image", "ok");
}

void
TexturedQuadDisplay::reset()
{
    Display::reset();
    texture_->clear();
    context_->queueRender();
    setStatus(StatusProperty::Warn, "Image", "No Image received");
}

/*
 *  protected member functions
 */
void
TexturedQuadDisplay::subscribe()
{
    if (!isEnabled())
    {
	return;
    }

    if (!image_topic_property_->getTopic().isEmpty())
    {
	try
	{
	    image_sub_ = nh_.subscribe(image_topic_property_->getTopicStd(), 1,
				       &TexturedQuadDisplay::updateImage,
				       this);
	    setStatus(StatusProperty::Ok, "Display Images Topic", "ok");
	}
	catch (ros::Exception& e)
	{
	    setStatus(StatusProperty::Error, "Display Images Topic",
		      QString("Error subscribing: ") + e.what());
	}
    }

    if (!quad_topic_property_->getTopic().isEmpty())
    {
	try
	{
	    quad_sub_ = nh_.subscribe(quad_topic_property_->getTopicStd(), 1,
				      &TexturedQuadDisplay::updateQuad, this);
	    setStatus(StatusProperty::Ok, "Quad Topic", "ok");
	}
	catch (ros::Exception& e)
	{
	    setStatus(StatusProperty::Error, "Quad Topic",
		      QString("Error subscribing: ") + e.what());
	}
    }
}

void
TexturedQuadDisplay::unsubscribe()
{
    image_sub_.shutdown();
    quad_sub_.shutdown();
}

/*
 *  private member functions
 */
void
TexturedQuadDisplay::updateImage(const image_cp& image)
{
    std::lock_guard<std::mutex>	lock(image_mutex_);

    cur_image_ = image;
}

void
TexturedQuadDisplay::updateQuad(const quad_cp& quad)
{
    std::lock_guard<std::mutex>	lock(quad_mutex_);

    cur_quad_ = quad;
}

void
TexturedQuadDisplay::createTexture()
{
    std::lock_guard<std::mutex>	lock(image_mutex_);

    const auto	img = cv_bridge::toCvCopy(*cur_image_,
					  sensor_msgs::image_encodings::RGBA8);
    if (!texture_)
	texture_.reset(new ROSImageTexture());
    texture_->addMessage(img->toImageMsg());
}

void
TexturedQuadDisplay::createMesh()
{
  // create our scenenode and material
    if (!mesh_node_)
    {
	const Ogre::String	resource_group_name = "MeshNode";

	if (auto&& rg_mgr = Ogre::ResourceGroupManager::getSingleton();
	    !rg_mgr.resourceGroupExists(resource_group_name))
	{
	    rg_mgr.createResourceGroup(resource_group_name);

	    mesh_material_ = Ogre::MaterialManager::getSingleton().create(
				resource_group_name + "MeshMaterial",
				resource_group_name);
	}

	mesh_node_.reset(this->scene_node_->createChildSceneNode());
    }

    if (!manual_object_)
    {
	manual_object_.reset(context_->getSceneManager()
				     ->createManualObject("MeshObject"));
	mesh_node_->attachObject(manual_object_.get());
    }

  // Lookup transform into fixed frame
    Ogre::Vector3	points[4];
    {
	std::lock_guard<std::mutex>	lock(quad_mutex_);

	Ogre::Vector3		position;
	Ogre::Quaternion	orientation;
	if (!context_->getFrameManager()
		     ->getTransform(cur_quad_->header.frame_id,
				    ros::Time(0), position, orientation))
	    throw std::runtime_error("Error transforming from fixed frame to frame " + cur_quad_->header.frame_id);

	Ogre::Matrix4	transform;
	transform.makeTransform(position, Ogre::Vector3(1.0, 1.0, 1.0),
				orientation);

	points[0] = transform.transformAffine(fromMsg(cur_quad_->top_left));
	points[1] = transform.transformAffine(fromMsg(cur_quad_->bottom_left));
	points[2] = transform.transformAffine(fromMsg(cur_quad_->bottom_right));
	points[3] = transform.transformAffine(fromMsg(cur_quad_->top_right));
    }

    auto	normal = (points[1] - points[0])
			.crossProduct(points[2] - points[1]);
    normal.normalise();

    manual_object_->clear();
    manual_object_->estimateVertexCount(4);
    manual_object_->begin(mesh_material_->getName(),
			  Ogre::RenderOperation::OT_TRIANGLE_LIST);
    manual_object_->position(points[0]);
    manual_object_->normal(normal);
    manual_object_->textureCoord(0, 0);
    manual_object_->position(points[1]);
    manual_object_->normal(normal);
    manual_object_->textureCoord(0, 1);
    manual_object_->position(points[2]);
    manual_object_->normal(normal);
    manual_object_->textureCoord(1, 1);
    manual_object_->position(points[3]);
    manual_object_->normal(normal);
    manual_object_->textureCoord(1, 0);
    manual_object_->quad(0, 1, 2, 3);
    manual_object_->end();
}

void
TexturedQuadDisplay::updateMeshProperties()
{
    const auto	pass = mesh_material_->getTechnique(0)->getPass(0);

    pass->setSelfIllumination(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
    pass->setDiffuse(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
    pass->setAmbient(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
    pass->setSpecular(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
    pass->setShininess(64.0f);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    context_->queueRender();
}

void
TexturedQuadDisplay::updateCamera()
{
    if (!mesh_node_ || mesh_material_.isNull())
	return;

    const auto	pass = mesh_material_->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);

    const auto	tex_state = pass->createTextureUnitState();  // "Decal.png");
    tex_state->setTextureName(texture_->getTexture()->getName());
    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR,
				   Ogre::FO_NONE);
    tex_state->setColourOperation(Ogre::LBO_REPLACE);  // don't accept addition
}

/*
 *  private member functions: Q_SLOTS
 */
void
TexturedQuadDisplay::updateDisplayImages()
{
    unsubscribe();
    subscribe();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::TexturedQuadDisplay, rviz::Display)
