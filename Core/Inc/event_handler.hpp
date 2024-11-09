#ifndef EVENT_HANDLER_HPP
#define EVENT_HANDLER_HPP

#include <optional>
#include <queue>

enum struct Event {
    TIMER_ELAPSED,
    PRINT_DUTKIEWICZ,
    PRINT_DUPA,
    PRINT_KARDYS,
    PRINT_BERNAT,
    PRINT_BOLTRUKIEWICZ,
    NONE,
};

struct EventHandler {
    void set_event(const Event event) noexcept
    {
        // const std::lock_guard<std::mutex> lock{this->mutex_};
        this->events_.push(event);
    }

    [[nodiscard]] std::optional<Event> get_event() noexcept
    {
        // const std::lock_guard<std::mutex> lock{this->mutex_};
        if (this->events_.empty()) {
            return std::optional<Event>{std::nullopt};
        }
        auto event{this->events_.front()};
        this->events_.pop();
        return std::optional<Event>{event};
    }

    [[nodiscard]] bool has_event() const noexcept
    {
        // const std::lock_guard<std::mutex> lock{this->mutex_};
        return !this->events_.empty();
    }

private:
    // mutable std::mutex mutex_{};
    std::queue<Event> events_{};
};

#endif // EVENT_HANDLER_HPP