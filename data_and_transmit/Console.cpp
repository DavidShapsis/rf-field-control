#include "Console.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

const char *ArgList::next() {
  while (*p_ == ' ' || *p_ == '\t') p_++;
  if (*p_ == 0) return nullptr;
  char *word = p_;
  while (*p_ && *p_ != ' ' && *p_ != '\t') p_++;
  if (*p_) *p_++ = 0;
  return word;
}

bool ArgList::nextInt(int lo, int hi, int &out) {
  const char *w = next();
  if (!w) return false;
  char *end;
  long v = strtol(w, &end, 10);
  if (end == w || *end != 0 || v < lo || v > hi) return false;
  out = (int)v;
  return true;
}

bool ArgList::done() {
  while (*p_ == ' ' || *p_ == '\t') p_++;
  return *p_ == 0;
}

bool Console::add(ConsoleCommand *cmd) {
  if (count_ >= MAX_COMMANDS) return false;
  commands_[count_++] = cmd;
  return true;
}

void Console::update() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len_ > 0) {
        line_[len_] = 0;
        execute(line_);
        len_ = 0;
      }
    } else if (len_ < LINE_LEN - 1) {
      line_[len_++] = tolower(c);
    }
  }
}

void Console::execute(char *line) {
  Serial.print(F("> "));
  Serial.println(line);                    // echo before ArgList splits the line
  ArgList args(line);
  const char *word = args.next();
  if (!word) return;

  if (strcmp_P(word, PSTR("help")) == 0 || strcmp(word, "?") == 0) {
    printHelp();
    return;
  }
  for (uint8_t i = 0; i < count_; i++) {
    if (strcmp_P(word, (PGM_P)commands_[i]->name()) == 0) {
      if (!commands_[i]->run(args)) {
        Serial.print(F("Usage: "));
        Serial.println(commands_[i]->usage());
      }
      return;
    }
  }
  Serial.println(F("Unknown command. Type help."));
}

void Console::printHelp() const {
  Serial.println(F("Commands:"));
  for (uint8_t i = 0; i < count_; i++) {
    Serial.print(F("  "));
    Serial.println(commands_[i]->usage());
  }
}
