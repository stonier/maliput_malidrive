// Copyright 2020 Toyota Research Institute

#include <memory>

#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::Rule;

/// Functor to build a DiscreteValueRuleStateProvider.
class DiscreteValueRuleStateProviderBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValueRuleStateProviderBuilder)

  /// Constructs a DiscreteValueRuleStateProviderBuilder.
  ///
  /// @param rulebook A RoadRulebook to feed the DiscreteValueRuleStateProvider.
  ///        It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When `rulebook` is nullptr.
  explicit DiscreteValueRuleStateProviderBuilder(const maliput::api::rules::RoadRulebook* rulebook)
      : rulebook_(rulebook) {
    MALIDRIVE_DEMAND(rulebook_ != nullptr);
  }

  /// Builds a DiscreteValueRuleStateProvider.
  std::unique_ptr<maliput::api::rules::DiscreteValueRuleStateProvider> operator()() const;

 private:
  const maliput::api::rules::RoadRulebook* rulebook_{};
};

}  // namespace builder
}  // namespace malidrive
