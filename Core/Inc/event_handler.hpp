#ifndef EVENT_HANDLER_HPP
#define EVENT_HANDLER_HPP

#include <cstdlib>
#include <optional>
#include <queue>

enum struct Event {
    TIMER_ELAPSED,
    DUTKIEWICZ,
    DUPA,
    KARDYS,
    BERNAT,
    BOLTRUKIEWICZ,
    KOZIER,
    NONE,
};

auto get_random_event()
{
    static auto seed{[i = 0]() mutable { std::srand(++i % INT32_MAX); }};
    seed();
    switch (std::rand() % 6 + 1) {
        case 1:
            return Event::BERNAT;
        case 2:
            return Event::DUPA;
        case 3:
            return Event::DUTKIEWICZ;
        case 4:
            return Event::BOLTRUKIEWICZ;
        case 5:
            return Event::KARDYS;
        case 6:
            return Event::KOZIER;
        default:
            return Event::NONE;
    }
}

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