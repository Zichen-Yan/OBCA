/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: app_ui_park.c
  Source: app_ui_park.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "app_ui_park.h"

static const uint32_t byd_interfaces_msg_app_ui_park_ops [] =
{
  /* app_ui_park */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_app_ui_park, APAStatus),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_app_ui_park, APA_SelctedNum_enum),
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_STU, offsetof (byd_interfaces_msg_app_ui_park, ParkingToDisplay), 6u, (5u << 16u) + 6u /* ParkingToDispaly */, sizeof (apa_msgs_msg_ParkingToDispaly),
  DDS_OP_RTS,

  /* ParkingToDispaly */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_ParkingToDispaly, Num),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P0_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P0_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P1_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P1_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P2_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P2_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P3_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, P3_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_ParkingToDispaly, Width),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_ParkingToDispaly, ParkingSpaceValid),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_ParkingToDispaly, ParkingSpaceAvailable),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_ParkingToDispaly, ParkingSpaceType),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_app_ui_park_desc =
{
  .m_size = sizeof (byd_interfaces_msg_app_ui_park),
  .m_align = 4u,
  .m_flagset = DDS_TOPIC_NO_OPTIMIZE | DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::app_ui_park_",
  .m_keys = NULL,
  .m_nops = 18,
  .m_ops = byd_interfaces_msg_app_ui_park_ops,
  .m_meta = ""
};

