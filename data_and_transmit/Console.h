// Console: reads lines from the Serial Monitor and runs ConsoleCommand objects.
#pragma once
#include <Arduino.h>

// Splits a line into space-separated words.
class ArgList {
public:
  explicit ArgList(char *line) : p_(line) {}
  const char *next();                                   // nullptr when there are no more
  bool nextInt(int lo, int hi, int &out);               // false if missing, not a number, or out of range
  bool done();                                          // true if nothing is left
private:
  char *p_;
};

class ConsoleCommand {
public:
  virtual ~ConsoleCommand() {}
  virtual const __FlashStringHelper *name() const = 0;
  virtual const __FlashStringHelper *usage() const = 0;   // shown in help and on bad input
  // Runs with the words after the command name. Return false to print the usage line.
  virtual bool run(ArgList &args) = 0;
};

class Console {
public:
  static const uint8_t MAX_COMMANDS = 16;
  static const uint8_t LINE_LEN = 48;

  bool add(ConsoleCommand *cmd);
  void update();                                        // call every loop
  void printHelp() const;

private:
  void execute(char *line);
  ConsoleCommand *commands_[MAX_COMMANDS];
  uint8_t count_ = 0;
  char line_[LINE_LEN];
  uint8_t len_ = 0;
};
