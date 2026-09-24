// log_capture.cpp — see log_capture.h.
#include "log_capture.h"

#include <deque>
#include <iostream>
#include <mutex>

namespace {
constexpr size_t kMaxLines = 1000;
std::mutex g_mutex;
std::deque<std::string> g_lines;
size_t g_total = 0;
} // namespace

LogCapture::LogCapture() {
    original_ = std::cerr.rdbuf();
    std::cerr.rdbuf(this);
}

LogCapture::~LogCapture() {
    std::cerr.rdbuf(original_);
}

void LogCapture::putChar(char c) {
    if (c == '\n') {
        original_->sputn(line_.data(), static_cast<std::streamsize>(line_.size()));
        original_->sputc('\n');
        g_lines.push_back(std::move(line_));
        if (g_lines.size() > kMaxLines) g_lines.pop_front();
        ++g_total;
        line_.clear();
    } else if (c != '\r') {
        line_ += c;
    }
}

int LogCapture::overflow(int c) {
    if (c == traits_type::eof()) return traits_type::not_eof(c);
    std::lock_guard<std::mutex> lk(g_mutex);
    putChar(static_cast<char>(c));
    return c;
}

std::streamsize LogCapture::xsputn(const char* s, std::streamsize n) {
    std::lock_guard<std::mutex> lk(g_mutex);
    for (std::streamsize i = 0; i < n; ++i) putChar(s[i]);
    return n;
}

std::vector<std::string> logSnapshot() {
    std::lock_guard<std::mutex> lk(g_mutex);
    return {g_lines.begin(), g_lines.end()};
}

size_t logLineCount() {
    std::lock_guard<std::mutex> lk(g_mutex);
    return g_total;
}
