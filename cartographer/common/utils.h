/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_UTILS_H_
#define CARTOGRAPHER_COMMON_UTILS_H_

namespace cartographer {
namespace common {

template <typename MapType, typename KeyType = typename MapType::key_type,
          typename ValueType = typename MapType::mapped_type>
ValueType* FindOrNull(MapType& map, const KeyType& key) {
  auto it = map.find(key);
  if (it == map.end()) return nullptr;
  return &(it->second);
}

}  // namespace common
}  // namespace cartographer

// qiao@2018.10.08: gpal debug utils
#include <chrono>
#include <glog/logging.h>

#define TRACE                       \
  do {                              \
    LOG(WARNING) << __FILE__ << ":" \
                 << __func__ << ":" \
                 << __LINE__;       \
  } while (0);
#if 0 // enable debug functions
#define FUNC_STAT_BEGIN                                   \
  static int s_##__func__##_called = 0;                   \
  std::chrono::steady_clock::time_point                   \
    __func__##begin, __func__##end;                       \
  __func__##begin = std::chrono::steady_clock::now();

#define FUNC_STAT_END                                       \
  __func__##end   = std::chrono::steady_clock::now();       \
  s_##__func__##_called++;                                  \
  LOG(WARNING) << __func__ << ":" << __LINE__ << " #" << s_##__func__##_called \
               << " exec time: " << std::chrono::duration_cast<std::chrono::microseconds>(__func__##end - __func__##begin).count() << " us";
#else
#define FUNC_STAT_BEGIN
#define FUNC_STAT_END
#endif // enable debug functions

#endif  // CARTOGRAPHER_COMMON_UTILS_H_
