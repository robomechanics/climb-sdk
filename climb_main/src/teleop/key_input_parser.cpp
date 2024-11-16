#include "climb_main/teleop/key_input_parser.hpp"
#include <sstream>
#include <iterator>

bool KeyInputParser::ChoiceToken::validate(const std::string & input) const
{
  for (const auto & token : tokens) {
    if (token->validate(input)) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> KeyInputParser::ChoiceToken::getOptions() const
{
  std::vector<std::string> options;
  for (const auto & token : tokens) {
    auto opt = token->getOptions();
    options.insert(options.end(), opt.begin(), opt.end());
  }
  return options;
}

std::string KeyInputParser::ChoiceToken::getName() const
{
  if (!name.empty()) {
    return name;
  }
  auto ss = std::stringstream();
  ss << "[";
  for (size_t i = 0; i < tokens.size(); ++i) {
    if (i > 0) {
      ss << "|";
    }
    ss << tokens.at(i)->getName();
  }
  ss << "]";
  return ss.str();
}

bool KeyInputParser::DoubleToken::validate(const std::string & input) const
{
  try {
    std::stod(input);
    return true;
  } catch (const std::invalid_argument & e) {
    return false;
  }
}

bool KeyInputParser::IntToken::validate(const std::string & input) const
{
  try {
    std::stoi(input);
    return true;
  } catch (const std::invalid_argument & e) {
    return false;
  }
}

bool KeyInputParser::BoolToken::validate(const std::string & input) const
{
  return input == "true" || input == "false";
}

KeyInputParser::Command::operator std::string() const
{
  auto ss = std::stringstream();
  for (size_t i = 0; i < tokens.size(); ++i) {
    if (i > 0) {
      ss << " ";
    }
    ss << tokens.at(i)->getName();
  }
  return ss.str();
}

KeyInputParser::KeyInputParser()
{
  // Basic token types
  Token string_token;
  DoubleToken double_token;
  IntToken int_token;
  BoolToken bool_token;
  tokens_[string_token.getName()] = std::make_shared<Token>();
  tokens_[double_token.getName()] = std::make_shared<DoubleToken>();
  tokens_[int_token.getName()] = std::make_shared<IntToken>();
  tokens_[bool_token.getName()] = std::make_shared<BoolToken>();

  // Default input callback function
  input_callback_ = [](const char & key) {
      return Response{"Received key: " + std::string(1, key), false};
    };

  // Help command
  defineCommand(
    "help", [this](
      [[maybe_unused]] const std::vector<std::string> & tokens)
    {
      return Response{help(), false};
    });
}

void KeyInputParser::defineToken(
  const std::string & name, const std::vector<std::string> & options)
{
  std::vector<std::shared_ptr<Token>> tokens;
  for (const auto & option : options) {
    tokens.push_back(std::make_shared<ConstantToken>(option));
  }
  tokens_[name] = std::make_shared<ChoiceToken>(name, tokens);
}

void KeyInputParser::defineToken(
  const std::string & name, const std::function<std::vector<std::string>()> & callback)
{
  tokens_[name] = std::make_shared<LazyToken>(name, callback);
}

std::shared_ptr<KeyInputParser::Token> KeyInputParser::getToken(const std::string & name)
{
  if (tokens_.find(name) != tokens_.end()) {
    return tokens_[name];
  }
  if (name.front() == '[' && name.back() == ']') {
    std::vector<std::shared_ptr<Token>> tokens;
    std::istringstream iss(name.substr(1, name.size() - 2));
    std::string option;
    while (std::getline(iss, option, '|')) {
      tokens.push_back(getToken(option));
    }
    return std::make_shared<ChoiceToken>(tokens);
  }
  return std::make_shared<ConstantToken>(name);
}

void KeyInputParser::defineCommand(
  const std::string & token_string,
  const std::function<KeyInputParser::Response(
    const std::vector<std::string> &)> & callback)
{
  auto ss = std::istringstream(token_string);
  std::vector<std::string> tokens(
    std::istream_iterator<std::string>{ss},
    std::istream_iterator<std::string>());
  Command command;
  for (const auto & token : tokens) {
    command.tokens.push_back(getToken(token));
  }
  command.callback = callback;
  // More recently added commands take precedence
  commands_.insert(commands_.begin(), command);
}

std::string KeyInputParser::complete(const std::vector<std::string> & tokens)
{
  std::vector<std::string> completions;
  for (const auto & command : commands_) {
    if (tokens.size() > command.tokens.size()) {
      continue;
    }
    bool valid = true;
    for (size_t i = 0; i < tokens.size() - 1; ++i) {
      if (!command.tokens.at(i)->validate(tokens.at(i))) {
        valid = false;
        break;
      }
    }
    if (!valid) {
      continue;
    }
    for (const auto & option :
      command.tokens.at(tokens.size() - 1)->getOptions())
    {
      if (option.find(tokens.back()) == 0) {
        auto ss = std::ostringstream{};
        for (size_t i = 0; i < tokens.size() - 1; ++i) {
          ss << tokens.at(i) << " ";
        }
        ss << option;
        if (tokens.size() < command.tokens.size()) {
          ss << " ";
        }
        completions.push_back(ss.str());
      }
    }
  }
  if (completions.empty()) {
    return "";
  }
  // Return longest sub-string that matches all possible completions
  for (size_t i = 0; i < completions.front().size(); ++i) {
    for (const auto & completion : completions) {
      if (completion.size() <= i ||
        completion.at(i) != completions.front().at(i))
      {
        return completions.front().substr(0, i);
      }
    }
  }
  return completions.front();
}

KeyInputParser::Response KeyInputParser::execute(
  const std::vector<std::string> & tokens)
{
  for (const auto & command : commands_) {
    if (tokens.size() != command.tokens.size()) {
      continue;
    }
    bool valid = true;
    for (size_t i = 0; i < tokens.size(); ++i) {
      if (!command.tokens.at(i)->validate(tokens.at(i))) {
        valid = false;
        break;
      }
    }
    if (valid) {
      return command.callback(tokens);
    }
  }
  return Response{"Invalid command", false};
}

KeyInputParser::Response KeyInputParser::input(const char key)
{
  return input_callback_(key);
}

KeyInputParser::Response KeyInputParser::processKeyInput(
  const std::string & input, bool realtime, bool autocomplete)
{
  if (input.empty()) {
    return Response{"", realtime};
  }
  std::istringstream iss(input);
  std::vector<std::string> tokens(
    std::istream_iterator<std::string>{iss},
    std::istream_iterator<std::string>());
  if (autocomplete) {
    auto response = complete(tokens);
    if (response.empty()) {
      return Response{input, false};
    }
    return Response{response, false};
  }
  if (realtime) {
    return this->input(tokens.front().front());
  }
  return execute(tokens);
}

std::string KeyInputParser::help()
{
  auto ss = std::stringstream();
  ss << "Command Syntax:";
  for (const auto & command : commands_) {
    ss << std::endl << "    " << std::string(command);
  }
  return ss.str();
}
