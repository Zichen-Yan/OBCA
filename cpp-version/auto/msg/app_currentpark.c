/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: app_currentpark.c
  Source: app_currentpark.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "app_currentpark.h"

static const uint32_t byd_interfaces_msg_app_currentpark_ops [] =
{
  /* app_currentpark */
  DDS_OP_ADR | DDS_OP_TYPE_EXT, offsetof (byd_interfaces_msg_app_currentpark, CurrentInfo), (3u << 16u) + 4u /* CurrentInfo */,
  DDS_OP_RTS,

  /* CurrentInfo */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_CurrentInfo, ID),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P0_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P0_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P1_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P1_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P2_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P2_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P3_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P3_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, Width),
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (byd_interfaces_msg_CurrentInfo, Angle),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_CurrentInfo, ParkingSpaceAvailable),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_CurrentInfo, ParkingSpaceType),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_app_currentpark_desc =
{
  .m_size = sizeof (byd_interfaces_msg_app_currentpark),
  .m_align = 8u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::app_currentpark_",
  .m_keys = NULL,
  .m_nops = 16,
  .m_ops = byd_interfaces_msg_app_currentpark_ops,
  .m_meta = ""
};

