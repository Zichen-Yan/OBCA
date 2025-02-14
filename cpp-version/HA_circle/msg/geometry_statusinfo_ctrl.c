/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: geometry_statusinfo_ctrl.c
  Source: geometry_statusinfo_ctrl.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "geometry_statusinfo_ctrl.h"

static const uint32_t byd_interfaces_msg_geometry_statusinfo_ctrl_ops [] =
{
  /* geometry_statusinfo_ctrl */
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_geometry_statusinfo_ctrl, TimeStampMs),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_geometry_statusinfo_ctrl, PlanningStatus),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_geometry_statusinfo_ctrl_desc =
{
  .m_size = sizeof (byd_interfaces_msg_geometry_statusinfo_ctrl),
  .m_align = 8u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::geometry_statusinfo_ctrl_",
  .m_keys = NULL,
  .m_nops = 3,
  .m_ops = byd_interfaces_msg_geometry_statusinfo_ctrl_ops,
  .m_meta = ""
};

