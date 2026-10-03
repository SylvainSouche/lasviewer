// log_capture.h — mirrors std::cerr into an in-memory buffer for the UI log.
#pragma once
#include <streambuf>
#include <string>
#include <vector>

// While alive, every line written to std::cerr (from any thread) is kept in
// a bounded buffer and still forwarded to the real stderr.
class LogCapture : public std::streambuf {
  public:
    LogCapture();
    ~LogCapture() override;

  protected:
    int overflow(int c) override;
    std::streamsize xsputn(const char* s, std::streamsize n) override;

  private:
    void putChar(char c);
    std::streambuf* original_;
    std::string line_;
};

// Thread-safe copy of the captured lines, oldest first.
std::vector<std::string> logSnapshot();
// Number of lines captured so far (monotonic; for change detection).
size_t logLineCount();
