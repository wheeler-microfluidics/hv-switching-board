#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <BaseNodeRpc.h>
#include <BaseNodeEeprom.h>
#include <BaseNodeI2c.h>
#include <BaseNodeConfig.h>
#include <BaseNodeSerialHandler.h>
#include <BaseNodeI2cHandler.h>
#include <Array.h>
#include <I2cHandler.h>
#include <SerialHandler.h>
#include "hv_switching_board_config_validate.h"
#include "hv_switching_board_config_pb.h"


namespace hv_switching_board {
const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC

class Node;

typedef nanopb::EepromMessage<hv_switching_board_Config,
                              config_validate::Validator<Node> > config_t;

class Node :
  public BaseNode,
  public BaseNodeEeprom,
  public BaseNodeI2c,
  public BaseNodeConfig<config_t>,
#ifndef DISABLE_SERIAL
  public BaseNodeSerialHandler,
#endif  // #ifndef DISABLE_SERIAL
  public BaseNodeI2cHandler {
public:
  typedef PacketParser<FixedPacket> parser_t;

  static const uint16_t BUFFER_SIZE = 128;  // >= longest property string
  static const uint16_t CHANNEL_COUNT = 40;

  uint8_t buffer_[BUFFER_SIZE];

  /* # Channels #
   *
   *  - Channels marked as *active* are cycled through during an actuation
   *    period.
   *  - Each active channel is assigned a level between 0 and 255.
   *  - The active channel mask provides constant-time (i.e., O(1)) lookup to
   *    determine whether a particular channel is currently *active*.
   *    * Without the `active_channel_mask_`, the `active_channel_index_` array
   *      must be traversed to determine if a particular channel is active
   *      (i.e., O(n)).
   *
   * # Global context #
   *
   * Note, that there may be more than one switching board on the I2C bus.
   * In this case during each actuation period, the actuation of active
   * channels across *all* boards on the bus must be coordinated.
   *
   * ## Coordination of active channel cycling across boards ##
   *
   * Let us define the *activation sequence* as a sequence of steps of
   * coordinated actuation across all boards on the bus.
   *
   * Prior to the start of an activation sequence, each switching board MUST
   * store the following information:
   *
   *  - The combined number of *active* channels across *all* switching boards
   *    on the bus (see `global_active_count_`).
   *  - The starting index of the board in the order of the global active
   *    channel list (see `global_start_index_`).
   *
   * During each step of an activation period, each switching board MUST be
   * aware of the following:
   *
   *  - The index (starting at 0) of the current activation step.
   */
  uint8_t active_channel_mask_[CHANNEL_COUNT / 8];  // 8 channels per byte
  uint8_t global_active_count_;  // Total active channels count (all boards)
  uint8_t global_start_index_;  // Activation sequence index of local channel 0
  uint8_t active_count_;  // Number of locally active channels (this board)
  uint8_t active_channel_index_[CHANNEL_COUNT];  // Local active channel ids
  uint8_t active_channel_level_[CHANNEL_COUNT];  // Local active levels

  Node() : BaseNode(),
           BaseNodeConfig<config_t>(hv_switching_board_Config_fields),
           global_active_count_(0), global_start_index_(0), active_count_(0) {}

  UInt8Array get_buffer() { return UInt8Array(sizeof(buffer_), buffer_); }
  /* This is a required method to provide a temporary buffer to the
   * `BaseNode...` classes. */

  void begin();
  void set_i2c_address(uint8_t value);  // Override to validate i2c address

  UInt8Array channel_levels() {
    UInt8Array output = get_buffer();
    output.length = CHANNEL_COUNT;
    for (int i = 0; i < CHANNEL_COUNT; i++) { output.data[i] = 0; }
    for (int i = 0; i < active_count_; i++) {
      output.data[active_channel_index_[i]] = active_channel_level_[i];
    }
    return output;
  }
  void reset_channels() {
    // Clear active channel mask.
    memset(active_channel_mask_, 0, sizeof(active_channel_mask_));
    active_count_ = 0;
  }
  UInt8Array active_mask() {
    UInt8Array output;
    output.data = active_channel_mask_;
    output.length = sizeof(active_channel_mask_);
    return output;
  }
  bool set_active_channels(UInt8Array indexes, UInt8Array levels) {
    if ((indexes.length == levels.length) &&
        (indexes.length <= CHANNEL_COUNT)) {
      reset_channels();
      active_count_ = indexes.length;
      for (int i = 0; i < indexes.length; i++) {
        const uint8_t channel_index = indexes.data[i];
        active_channel_index_[i] = channel_index;
        active_channel_level_[i] = levels.data[i];
        // Mark channel as active in mask.
        const uint8_t bit_index = 7 - (channel_index % 8);
        active_channel_mask_[channel_index / 8] |= 1 << bit_index;
      }
      return true;
    }
    return false;
  }

  uint8_t active_count() const { return active_count_; }
  uint8_t channel_count() const { return CHANNEL_COUNT; }

  uint8_t global_active_count() const { return global_active_count_; }
  uint8_t global_start_index() const { return global_start_index_; }
  uint8_t set_global_active_count(uint8_t value) { global_active_count_ = value; }
  uint8_t set_global_start_index(uint8_t value) { global_start_index_ = value; }
};

}  // namespace hv_switching_board


#endif  // #ifndef ___NODE__H___
