/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: Freespacecell.c
  Source: Freespacecell.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "Freespacecell.h"

static const uint32_t uss_msgs_msg_FreeSpaceCell_ops [] =
{
  /* FreeSpaceCell */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (uss_msgs_msg_FreeSpaceCell, Status),
  DDS_OP_ADR | DDS_OP_TYPE_2BY, offsetof (uss_msgs_msg_FreeSpaceCell, Probability),
  DDS_OP_RTS
};

const dds_topic_descriptor_t uss_msgs_msg_FreeSpaceCell_desc =
{
  .m_size = sizeof (uss_msgs_msg_FreeSpaceCell),
  .m_align = 2u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "uss_msgs::msg::dds_::FreeSpaceCell_",
  .m_keys = NULL,
  .m_nops = 3,
  .m_ops = uss_msgs_msg_FreeSpaceCell_ops,
  .m_meta = ""
};

