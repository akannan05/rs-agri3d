// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rslidar_msg:msg/RslidarPacket.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rslidar_msg/msg/rslidar_packet.h"


#ifndef RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__FUNCTIONS_H_
#define RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "rslidar_msg/msg/rosidl_generator_c__visibility_control.h"

#include "rslidar_msg/msg/detail/rslidar_packet__struct.h"

/// Initialize msg/RslidarPacket message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rslidar_msg__msg__RslidarPacket
 * )) before or use
 * rslidar_msg__msg__RslidarPacket__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
bool
rslidar_msg__msg__RslidarPacket__init(rslidar_msg__msg__RslidarPacket * msg);

/// Finalize msg/RslidarPacket message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
void
rslidar_msg__msg__RslidarPacket__fini(rslidar_msg__msg__RslidarPacket * msg);

/// Create msg/RslidarPacket message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rslidar_msg__msg__RslidarPacket__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
rslidar_msg__msg__RslidarPacket *
rslidar_msg__msg__RslidarPacket__create(void);

/// Destroy msg/RslidarPacket message.
/**
 * It calls
 * rslidar_msg__msg__RslidarPacket__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
void
rslidar_msg__msg__RslidarPacket__destroy(rslidar_msg__msg__RslidarPacket * msg);

/// Check for msg/RslidarPacket message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
bool
rslidar_msg__msg__RslidarPacket__are_equal(const rslidar_msg__msg__RslidarPacket * lhs, const rslidar_msg__msg__RslidarPacket * rhs);

/// Copy a msg/RslidarPacket message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
bool
rslidar_msg__msg__RslidarPacket__copy(
  const rslidar_msg__msg__RslidarPacket * input,
  rslidar_msg__msg__RslidarPacket * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
const rosidl_type_hash_t *
rslidar_msg__msg__RslidarPacket__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
const rosidl_runtime_c__type_description__TypeDescription *
rslidar_msg__msg__RslidarPacket__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
const rosidl_runtime_c__type_description__TypeSource *
rslidar_msg__msg__RslidarPacket__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
const rosidl_runtime_c__type_description__TypeSource__Sequence *
rslidar_msg__msg__RslidarPacket__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/RslidarPacket messages.
/**
 * It allocates the memory for the number of elements and calls
 * rslidar_msg__msg__RslidarPacket__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
bool
rslidar_msg__msg__RslidarPacket__Sequence__init(rslidar_msg__msg__RslidarPacket__Sequence * array, size_t size);

/// Finalize array of msg/RslidarPacket messages.
/**
 * It calls
 * rslidar_msg__msg__RslidarPacket__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
void
rslidar_msg__msg__RslidarPacket__Sequence__fini(rslidar_msg__msg__RslidarPacket__Sequence * array);

/// Create array of msg/RslidarPacket messages.
/**
 * It allocates the memory for the array and calls
 * rslidar_msg__msg__RslidarPacket__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
rslidar_msg__msg__RslidarPacket__Sequence *
rslidar_msg__msg__RslidarPacket__Sequence__create(size_t size);

/// Destroy array of msg/RslidarPacket messages.
/**
 * It calls
 * rslidar_msg__msg__RslidarPacket__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
void
rslidar_msg__msg__RslidarPacket__Sequence__destroy(rslidar_msg__msg__RslidarPacket__Sequence * array);

/// Check for msg/RslidarPacket message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
bool
rslidar_msg__msg__RslidarPacket__Sequence__are_equal(const rslidar_msg__msg__RslidarPacket__Sequence * lhs, const rslidar_msg__msg__RslidarPacket__Sequence * rhs);

/// Copy an array of msg/RslidarPacket messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rslidar_msg
bool
rslidar_msg__msg__RslidarPacket__Sequence__copy(
  const rslidar_msg__msg__RslidarPacket__Sequence * input,
  rslidar_msg__msg__RslidarPacket__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__FUNCTIONS_H_
