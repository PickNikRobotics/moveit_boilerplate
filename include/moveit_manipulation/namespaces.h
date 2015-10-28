/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/
/*
   Author: Dave Coleman <dave@dav.ee>
   Desc:   Shortcuts for namespaces
*/

#ifndef MOVEIT_MANIPULATION__NAMESPACES
#define MOVEIT_MANIPULATION__NAMESPACES

// Temporarily define namespaces
namespace moveit_visual_tools
{
}
namespace rviz_visual_tools
{
}
namespace ompl_visual_tools
{
}
namespace moveit
{
namespace core
{
class JointModelGroup;
}
}

// Shortcuts
namespace mvt = moveit_visual_tools;
namespace rvt = rviz_visual_tools;
namespace ovt = ompl_visual_tools;

typedef const moveit::core::JointModelGroup JointModelGroup;

#endif
