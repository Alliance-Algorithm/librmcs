#include <atomic>
#include <thread>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/typing.h>

#include <librmcs/forwarder/cboard.hpp>

namespace py = pybind11;

struct PyAsyncObjectCollection {
    py::object asyncio = py::module_::import("asyncio");

    py::object is_coroutine_function = py::module_::import("inspect").attr("iscoroutinefunction");

    py::object event_loop = asyncio.attr("get_event_loop")();
    py::object run_coroutine_threadsafe = asyncio.attr("run_coroutine_threadsafe");
};

class Delegate {
public:
    void add(const PyAsyncObjectCollection& async, const py::function& callable) {
        auto new_call_struct = new LinkedCallStruct{
            async.is_coroutine_function(callable).cast<bool>(), callable, nullptr};

        auto call_struct = linked_call_struct_.load(std::memory_order::relaxed);
        if (!call_struct) {
            linked_call_struct_.store(new_call_struct, std::memory_order::relaxed);
        } else {
            auto last_call_struct = call_struct;
            while (auto next = last_call_struct->next.load(std::memory_order::relaxed))
                last_call_struct = next;
            last_call_struct->next.store(new_call_struct, std::memory_order::relaxed);
        }
    }

    template <typename... Args>
    void call(const PyAsyncObjectCollection& async, Args&&... args) {
        for (auto call_struct = linked_call_struct_.load(std::memory_order::relaxed); call_struct;
             call_struct = call_struct->next.load(std::memory_order::relaxed)) {
            if (call_struct->is_async) {
                auto awaitable = call_struct->function(std::forward<Args>(args)...);
                async.run_coroutine_threadsafe(awaitable, async.event_loop);
            } else {
                call_struct->function(std::forward<Args>(args)...);
            }
        }
    }

private:
    struct LinkedCallStruct {
        bool is_async;
        py::function function;
        std::atomic<LinkedCallStruct*> next;
    };
    std::atomic<LinkedCallStruct*> linked_call_struct_{nullptr};
};

class CBoard : librmcs::forwarder::CBoard {
public:
    explicit CBoard(uint16_t usb_pid)
        : librmcs::forwarder::CBoard(usb_pid)
        , event_thread_([this]() { handle_events(); }) {
        event_thread_.detach();
    }

    py::function can1_receive(const py::function& callable) {
        can1_receive_callback_.add(async_object_, callable);
        return callable;
    }

    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        py::gil_scoped_acquire acquire;
        can1_receive_callback_.call(async_object_, py::arg("can_data") = can_data);
    }

private:
    std::thread event_thread_;

    PyAsyncObjectCollection async_object_;

    Delegate can1_receive_callback_;
};

void spin() { py::module_::import("asyncio").attr("get_event_loop")().attr("run_forever")(); }

PYBIND11_MODULE(__librmcscpp, m) {
    m.attr("__name__") = "librmcspy.__librmcscpp";

    m.doc() = R"pbdoc(
        Cpp part of librmcspy, created by pybind11.
    )pbdoc";

    m.def("spin", &spin);

    // m.def(
    //     "subtract", [](int i, int j) { return i - j; }, R"pbdoc(
    //     Subtract two numbers

    //     Some other explanation about the subtract function.
    // )pbdoc");

    py::class_<CBoard>(m, "CBoard")
        .def(py::init<uint16_t>(), py::arg("usb_pid"))
        .def("can1_receive", &CBoard::can1_receive, py::arg("callable"));

#define STRINGIFY(x) #x
#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
