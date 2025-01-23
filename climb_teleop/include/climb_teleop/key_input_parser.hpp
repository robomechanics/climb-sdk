#ifndef CLIMB_TELEOP__KEY_INPUT_PARSER_HPP_
#define CLIMB_TELEOP__KEY_INPUT_PARSER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Parses strings of input from the keyboard
 *
 * Commands are defined by space-separated string of tokens, listed below.
 *
 * - "STRING": An arbitrary string
 *
 * - "DOUBLE": A double value
 *
 * - "INT": An integer value
 *
 * - "BOOL": A boolean value (true or false)
 *
 * - "value1": A specific value
 *
 * - "[value1|value2|...]": A choice between a list of values
 *
 * Users can also define custom tokens representing a choice between values,
 * either precomputed or evaluated by a callback when a command is processed.
 *
 * Example command: "set JOINT [position velocity effort] DOUBLE" where JOINT
 * is a custom token representing the list of joint names
 */
class KeyInputParser
{
public:
  /**
   * @brief Struct to store token definition
   */
  struct Token
  {
    /**
     * @brief Validate a possible value of the token
     */
    virtual bool validate([[maybe_unused]] const std::string & input) const
    {return true;}

    /**
     * @brief Get possible values for token (used for autocompletion)
     */
    virtual std::vector<std::string> getOptions() const {return {};}

    virtual std::string getName() const {return "STRING";}
  };

  struct ChoiceToken : Token
  {
    explicit ChoiceToken(const std::vector<std::shared_ptr<Token>> & tokens)
    : tokens(tokens) {}
    ChoiceToken(
      const std::string & name,
      const std::vector<std::shared_ptr<Token>> & tokens)
    : name(name), tokens(tokens) {}
    bool validate(const std::string & input) const override;
    std::vector<std::string> getOptions() const override;
    std::string getName() const override;
    std::string name;
    std::vector<std::shared_ptr<Token>> tokens;
  };

  struct LazyToken : ChoiceToken
  {
    LazyToken(
      const std::string & name,
      const std::function<std::vector<std::string>()> & callback)
    : ChoiceToken({}), name(name), callback(callback) {}
    bool validate(const std::string & input) const override
    {
      auto options = callback();
      return std::find(options.begin(), options.end(), input) != options.end();
    }
    std::vector<std::string> getOptions() const override
    {return callback();}
    std::string getName() const override {return name;}
    std::string name;
    std::function<std::vector<std::string>()> callback;
  };

  struct ConstantToken : Token
  {
    explicit ConstantToken(const std::string & value)
    : value(value) {}
    bool validate(const std::string & input) const override
    {return input == value;}
    std::vector<std::string> getOptions() const override {return {value};}
    std::string getName() const override {return value;}
    std::string value;
  };

  struct DoubleToken : Token
  {
    bool validate(const std::string & input) const override;
    std::string getName() const override {return "DOUBLE";}
  };

  struct IntToken : Token
  {
    bool validate(const std::string & input) const override;
    std::string getName() const override {return "INT";}
  };

  struct BoolToken : Token
  {
    bool validate(const std::string & input) const override;
    std::string getName() const override {return "BOOL";}
  };

  /**
   * @brief Struct to store response to a command
   */
  struct Response
  {
    std::string message;
    bool realtime;
  };

  /**
   * @brief Struct to store command definition and callback
   */
  struct Command
  {
    std::vector<std::shared_ptr<Token>> tokens;
    std::function<Response(const std::vector<std::string> &)> callback;
    operator std::string() const;
  };

  /**
   * @brief Constructor for KeyInputParser
   */
  KeyInputParser();

  /**
   * @brief Define a token with a list of possible values
   * @param name The unique identifier for the token
   * @param options The list of possible values
   */
  void defineToken(
    const std::string & name, const std::vector<std::string> & options);

  /**
   * @brief Define a token with a callback function to get possible values
   * @param name The unique identifier for the token
   * @param callback The callback function to get possible values
   */
  void defineToken(
    const std::string & name,
    const std::function<std::vector<std::string>()> & callback);

  /**
   * @brief Define a command with a list of tokens and a callback function
   * @param token_string The list of tokens for the command separated by spaces
   * @param callback The callback function for the command
   */
  void defineCommand(
    const std::string & token_string,
    const std::function<Response(
      const std::vector<std::string> &)> & callback);

  /**
   * @brief Set the callback function for realtime key input
   * @param callback The input callback function
   */
  void setInputCallback(
    const std::function<Response(const char)> & callback)
  {input_callback_ = callback;}

  /**
   * @brief Process a key input
   * @param input The key input string
   * @param realtime Input was sent in realtime mode
   * @param autocomplete Request to autocomplete the input string
   * @return The response to the input
   */
  Response processKeyInput(
    const std::string & input, bool realtime, bool autocomplete);

  /**
   * @brief Get a list of all valid commands
   * @return The help message
   */
  std::string help();

private:
  /**
   * @brief Get a token by name
   * @param name The name of the token
   */
  std::shared_ptr<KeyInputParser::Token> getToken(const std::string & name);

  /**
   * @brief Find valid autocompletion of a partial input string
   * @param tokens The tokenized input string
   * @return The autocompleted input string
   */
  std::string complete(const std::vector<std::string> & tokens);

  /**
   * @brief Execute a command
   * @param tokens The tokenized input string
   * @return The response to the command
   */
  Response execute(const std::vector<std::string> & tokens);

  /**
   * @brief Process a realtime key input
   * @param key The input key
   * @return The response to the input
   */
  Response input(const char key);

  std::unordered_map<std::string, std::shared_ptr<KeyInputParser::Token>>
  tokens_;
  std::vector<Command> commands_;
  std::function<Response(const char)> input_callback_;
};

#endif  // CLIMB_TELEOP__KEY_INPUT_PARSER_HPP_
