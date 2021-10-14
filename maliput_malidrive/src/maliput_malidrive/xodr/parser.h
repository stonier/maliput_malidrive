// Copyright 2020 Toyota Research Institute
#pragma once

#include <optional>

#include <tinyxml2.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/parser_configuration.h"

namespace malidrive {
namespace xodr {

/// Base class for parsing a tinyxml2::XMLElement node.
class ParserBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(ParserBase);
  ParserBase() = delete;
  virtual ~ParserBase() = default;

  /// Creates an ParserBase from a tinyxml2::XMLElement.
  /// @param element Is the XML Node that contains attributes to be parsed.
  /// @param parser_configuration Holds the configuration for the parser.
  /// @throw maliput::common::assertion_error When `element` is nullptr.
  ParserBase(tinyxml2::XMLElement* element, const ParserConfiguration& parser_configuration)
      : element_(element), parser_configuration_(parser_configuration) {
    MALIDRIVE_THROW_UNLESS(element_ != nullptr);
  }

  /// Count the number of attributes the tinyxml2::XMLElement has.
  /// @return The number of attributes.
  int NumberOfAttributes() const;

  /// @returns The element's name.
  std::string GetName() const { return static_cast<std::string>(element_->Value()); }

 protected:
  /// A XML node.
  tinyxml2::XMLElement* element_{};
  /// Parser configuration.
  ParserConfiguration parser_configuration_{};
};

/// Parses XML node's attributes descriptions.
/// There are specializations for:
/// - double
/// - std::string
class AttributeParser : public ParserBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(AttributeParser);
  AttributeParser() = delete;
  ~AttributeParser() = default;

  /// Creates an AttributeParser from a tinyxml2::XMLElement.
  /// @param element Is the XML Node that contains attributes to be parsed.
  /// @param parser_configuration Holds the configuration for the parser.
  /// @throw maliput::common::assertion_error When `element` is nullptr.
  AttributeParser(tinyxml2::XMLElement* element, const ParserConfiguration& parser_configuration)
      : ParserBase(element, parser_configuration) {}

  /// Parses the `attribute_name` as `T`.
  /// @tparam T Is the type to parse the attribute's value into.
  /// @param attribute_name Is the attribute to be parsed.
  /// @return A `T(value)` object when `attribute_name` is present, otherwise a std::nullopt.
  template <typename T>
  std::optional<T> As(const std::string& attribute_name) const;
};

/// Parses XML nodes.
/// There are specializations for:
/// - Header
/// - RoadHeader
class NodeParser : public ParserBase {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(NodeParser);
  NodeParser() = delete;
  ~NodeParser() = default;

  /// Creates a NodeParser from a tinyxml2::XMLElement.
  /// @param element Is the XML Node to be parsed.
  /// @param parser_configuration Holds the configuration for the parser.
  /// @throw maliput::common::assertion_error When `element` is nullptr.
  NodeParser(tinyxml2::XMLElement* element, const ParserConfiguration& parser_configuration)
      : ParserBase(element, parser_configuration) {}

  /// Parses the node as `T`.
  /// @tparam T Is the type of the node's value.
  /// @return A `T(value)`.
  /// @throw maliput::common::assertion_error When the node couldn't be parsed as `T`.
  template <typename T>
  T As() const;
};

/// Convert all the content of the XML node into text.
/// @param element Is the XML Node.
/// @returns A string with the XML description.
/// @throw maliput::common::assertion_error When element is nullptr.
std::string ConvertXMLNodeToText(tinyxml2::XMLElement* element);

/// Modifies `text` to adapt the message to be compatible with maliput::common::Logger.
/// @details Escapes formatting when having curly braces in the text that aren't meant to be formatted as arguments.
/// @param text Text to be adapted.
/// @throws maliput::common::assertion_error When text is nullptr.
void DuplicateCurlyBracesForFmtLogging(std::string* text);

}  // namespace xodr
}  // namespace malidrive
