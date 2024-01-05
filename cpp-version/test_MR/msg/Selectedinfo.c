/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: Selectedinfo.c
  Source: Selectedinfo.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "Selectedinfo.h"

static const uint32_t apa_msgs_msg_SelectedInfo_ops [] =
{
  /* SelectedInfo */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_SelectedInfo, SelectID),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P0_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P0_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P1_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P1_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P2_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P2_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P3_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, P3_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (apa_msgs_msg_SelectedInfo, Width),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_SelectedInfo, ParkingSpaceValid),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_SelectedInfo, ParkingSpaceAvailable),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (apa_msgs_msg_SelectedInfo, ParkingSpaceType),
  DDS_OP_RTS
};

const dds_topic_descriptor_t apa_msgs_msg_SelectedInfo_desc =
{
  .m_size = sizeof (apa_msgs_msg_SelectedInfo),
  .m_align = 4u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "apa_msgs::msg::dds_::SelectedInfo_",
  .m_keys = NULL,
  .m_nops = 14,
  .m_ops = apa_msgs_msg_SelectedInfo_ops,
  .m_meta = ""
};

