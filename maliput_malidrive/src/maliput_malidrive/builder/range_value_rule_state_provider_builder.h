// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <memory>

#include <maliput/api/rules/range_value_rule_state_provider.h>
#include <maliput/api/rules/road_rulebook.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

/// Functor to build a RangeValueRuleStateProvider.
class RangeValueRuleStateProviderBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RangeValueRuleStateProviderBuilder)

  /// Constructs a MalidriveRangeValueRuleStateProviderBuilder.
  ///
  /// @param rulebook A RoadRulebook to feed the RangeValueRuleStateProvider.
  ///        It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When `rulebook` is nullptr.
  explicit RangeValueRuleStateProviderBuilder(const maliput::api::rules::RoadRulebook* rulebook) : rulebook_(rulebook) {
    MALIDRIVE_DEMAND(rulebook_ != nullptr);
  }

  /// Builds a RangeValueRuleStateProvider.
  std::unique_ptr<maliput::api::rules::RangeValueRuleStateProvider> operator()() const;

 private:
  const maliput::api::rules::RoadRulebook* rulebook_{};
};

}  // namespace builder
}  // namespace malidrive
