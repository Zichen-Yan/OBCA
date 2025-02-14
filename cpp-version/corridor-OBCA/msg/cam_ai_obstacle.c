/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: cam_ai_obstacle.c
  Source: cam_ai_obstacle.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "cam_ai_obstacle.h"

static const uint32_t byd_interfaces_msg_cam_ai_obstacle_ops [] =
{
  /* cam_ai_obstacle */
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_cam_ai_obstacle, TimeStampMs),
  DDS_OP_ADR | DDS_OP_TYPE_EXT, offsetof (byd_interfaces_msg_cam_ai_obstacle, position), (3u << 16u) + 9u /* Position */,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_STU, offsetof (byd_interfaces_msg_cam_ai_obstacle, ObstacleInfo), 25u, (5u << 16u) + 13u /* ObstacleInfo */, sizeof (uss_msgs_msg_ObstacleInfo),
  DDS_OP_RTS,

  /* Position */
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (uss_msgs_msg_Position, X),
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (uss_msgs_msg_Position, Y),
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (uss_msgs_msg_Position, Heading),
  DDS_OP_RTS,

  /* ObstacleInfo */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (uss_msgs_msg_ObstacleInfo, ObstacleType),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (uss_msgs_msg_ObstacleInfo, Obstacle_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (uss_msgs_msg_ObstacleInfo, Obstacle_Y),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (uss_msgs_msg_ObstacleInfo, VehicleDirection),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_cam_ai_obstacle_desc =
{
  .m_size = sizeof (byd_interfaces_msg_cam_ai_obstacle),
  .m_align = 8u,
  .m_flagset = DDS_TOPIC_NO_OPTIMIZE | DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::cam_ai_obstacle_",
  .m_keys = NULL,
  .m_nops = 13,
  .m_ops = byd_interfaces_msg_cam_ai_obstacle_ops,
  .m_meta = ""
};

