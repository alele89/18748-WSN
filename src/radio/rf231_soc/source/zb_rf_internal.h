#define ZB_RF_PKT_16B_ZERO_BYTE 0
#define ZB_RF_PKT_16B_FIRST_BYTE 1

typedef unsigned char uchar_t;
typedef unsigned short uint_t; 

typedef struct zb_rf_mac_mhr_s
{
  uint8_t frame_control[2];
  uint8_t seq_number;
  uint16_t dst_pan_id;
  uint16_t dst_addr;
  uint16_t src_pan_id;
  uint16_t src_addr;

}
zb_rf_mac_mhr_t;


#define ZB_RF_FCF_SET_FRAME_TYPE( p_fcf, frame_type )                                 \
do                                                                                 \
{                                                                                  \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) &= 0xf8;                        \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) |= (uchar_t) ( frame_type ); \
} while( 0 )


/**
   Gets frame type subfield in frame control field ( FCF )
   Return values are in range of \see mac_frame_type_e enum.

   @param p_fcf - pointer to 16bit FCF field.
*/

#define ZB_RF_FCF_GET_FRAME_TYPE( p_fcf ) ((uint_t)( ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) & 0x07 ))


/**
   Gets security bit subfield in frame control field ( FCF )
   Return values can be 0 or 1.

   @param p_fcf - pointer to 16bit FCF field.
*/

#define ZB_RF_FCF_GET_SECURITY_BIT( p_fcf ) ((uint_t)( ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) & 0x08 ))


/**
   Sets security bit subfield in frame control field ( FCF )
   Input values can be 0 or 1.

   @param p_fcf     - pointer to 16bit FCF field.
   @param bit_value - 0 or 1.
*/

#define ZB_RF_FCF_SET_SECURITY_BIT( p_fcf, bit_value )                     \
  do                                                                    \
{                                                                       \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) &= 0xF7;             \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) |= (bit_value) << 3; \
} while( 0 )

/**
   Gets frame pending bit subfield in frame control field ( FCF )
   Return values can be 0 or 1.

   @param p_fcf - pointer to 16bit FCF field.
*/

#define ZB_RF_FCF_GET_FRAME_PENDING_BIT( p_fcf ) ((uint_t)(( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) & 0x10 ))


/**
   Sets security bit subfield in frame control field ( FCF )

   @param p_fcf     - pointer to 16bit FCF field.
   @param bit_value - 0 or 1.
*/

#define ZB_RF_FCF_SET_FRAME_PENDING_BIT( p_fcf, bit_value )                \
  do                                                                    \
{                                                                       \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) &= 0xEF;             \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) |= (bit_value) << 4; \
} while( 0 )

/**
   Gets ack request bit subfield in frame control field ( FCF )
   Return values can be 0 or 1.

   @param p_fcf - pointer to 16bit FCF field.
*/

#define ZB_RF_FCF_GET_ACK_REQUEST_BIT( p_fcf ) ((uint_t)( ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) & 0x20 ))


/**
   Sets ack request bit subfield in frame control field ( FCF )

   @param p_fcf     - pointer to 16bit FCF field.
   @param bit_value - 0 or 1.
*/

#define ZB_RF_FCF_SET_ACK_REQUEST_BIT( p_fcf, bit_value )                  \
  do                                                                    \
{                                                                       \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) &= 0xDF;             \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) |= (bit_value) << 5; \
} while( 0 )


/**
   Gets PAN ID compression bit subfield in frame control field ( FCF )
   Return values can be 0 or 1.

   @param p_fcf - pointer to 16bit FCF field.
*/

#define ZB_RF_FCF_GET_PANID_COMPRESSION_BIT( p_fcf ) ((uint_t)( ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) & 0x40 ))


/**
   Sets ack request bit subfield in frame control field ( FCF )

   @param p_fcf     - pointer to 16bit FCF field.
   @param bit_value - 0 or 1.
*/

#define ZB_RF_FCF_SET_PANID_COMPRESSION_BIT( p_fcf, bit_value )            \
  do                                                                    \
{                                                                       \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) &= 0xBF;             \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_ZERO_BYTE] ) |= (bit_value) << 6; \
} while( 0 )


/**
   Gets destination addressing mode subfield in frame control field ( FCF )
   Return values is one value from zb_rf_addr_mode_e enum.

   @param p_fcf - pointer to 16bit FCF field.
*/

#define ZB_RF_FCF_GET_FRAME_VERSION( p_fcf )  ( ((uint_t)( ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_FIRST_BYTE] ) & 0x30 )) >> 4 )


/**
   Sets frame version subfield in frame control field ( FCF )
   Return values is one value defined in mac_frame_version_e.

   @param p_fcf     - pointer to 16bit FCF field.
   @param frame_version -
*/

#define ZB_RF_FCF_SET_FRAME_VERSION( p_fcf, frame_version )        \
do                                                              \
{                                                               \
                                                                \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_FIRST_BYTE] ) &= 0xCF;                    \
  ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_FIRST_BYTE] ) |= ( frame_version ) << 4;  \
} while( 0 )

/**
   Gets source addressing mode subfield in frame control field ( FCF )
   Return values is one value from zb_rf_addr_mode_e enum.

   @param p_fcf - pointer to 16bit FCF field.
*/

#define ZB_RF_FCF_GET_SRC_ADDRESSING_MODE( p_fcf )  ( ((uint_t)( ( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_FIRST_BYTE] ) & 0xC0 )) >> 6 )

#define ZB_RF_FCF_GET_DST_ADDRESSING_MODE( p_fcf )  (( (uint_t)(( ( ( uint8_t* ) ( p_fcf ) )[ZB_RF_PKT_16B_FIRST_BYTE] ) & 0x0C) ) >> 2 )
