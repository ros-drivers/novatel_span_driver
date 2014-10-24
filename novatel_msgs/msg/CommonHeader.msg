# On the wire, this header is preceeded by three sync bytes,
# which are 0xAA 0x44 0x12, and a uint8 which is the header length.

# Message ID of the log being output.
uint16 id

# Measurement source, format, response bit.
uint8 msg_type

uint8 port_addr
uint8 length
uint8 sequence

uint16 idle_time   # float
uint16 time_stat

uint16 week
int32 gpssec

uint32 rcvrstat
uint16 reserved_h
uint16 version_sw

