/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: vehiclecontrol_plana_reqinfo.c
  Source: vehiclecontrol_plana_reqinfo.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "vehiclecontrol_plana_reqinfo.h"

static const uint32_t byd_interfaces_msg_vehiclecontrol_plana_reqinfo_ops [] =
{
  /* vehiclecontrol_plana_reqinfo */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_vehiclecontrol_plana_reqinfo, PlanningRequest),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (byd_interfaces_msg_vehiclecontrol_plana_reqinfo, ObsUssInfo),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_vehiclecontrol_plana_reqinfo_desc =
{
  .m_size = sizeof (byd_interfaces_msg_vehiclecontrol_plana_reqinfo),
  .m_align = 4u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::vehiclecontrol_plana_reqinfo_",
  .m_keys = NULL,
  .m_nops = 3,
  .m_ops = byd_interfaces_msg_vehiclecontrol_plana_reqinfo_ops,
  .m_meta = ""
};

