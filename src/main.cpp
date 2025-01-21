#include <iostream>
#include <cmath>
#include <algorithm>
#include <format>
#include <fstream>
#include <mutex>
#include <queue>
#include <semaphore>
#include <sys/signal.h>
#include <time.h>
#include <thread>
#include <wayland-client.h>
#include <utility>
#include <unistd.h>
#include <vector>
#include "protocols/hyprland-ctm-control-v1.hpp"
#include "protocols/wayland.hpp"

#include "helpers/Log.hpp"

#include <hyprutils/math/Mat3x3.hpp>
#include <hyprutils/memory/WeakPtr.hpp>
using namespace Hyprutils::Math;
using namespace Hyprutils::Memory;
#define SP CSharedPointer
#define WP CWeakPointer

using std::ofstream;
using std::pair;
using std::vector;
using std::string;
using std::stoi;

// The kelvin value we use that is close to the identity matrix (e.g. when interpolating)
static const int kIdentityKelvin = 6600;

// kindly borrowed from https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
static Mat3x3 matrixForKelvin(unsigned long long temperature) {
    float r = 1.F, g = 1.F, b = 1.F;

    double temp = (double)temperature / 100.0;

    if (temp <= 66) {
        r = 255;
        g = std::clamp(99.4708025861 * std::log(temp) - 161.1195681661, 0.0, 255.0);
        if (temp <= 19)
            b = 0;
        else
            b = std::clamp(std::log(temp - 10) * 138.5177312231 - 305.0447927307, 0.0, 255.0);
    } else {
        r = std::clamp(329.698727446 * (std::pow(temp - 60, -0.1332047592)), 0.0, 255.0);
        g = std::clamp(288.1221695283 * (std::pow(temp - 60, -0.0755148492)), 0.0, 255.0);
        b = 255;
    }

    return std::array<float, 9>{r / 255.F, 0, 0, 0, g / 255.F, 0, 0, 0, b / 255.F};
}

struct SOutput {
    SP<CCWlOutput> output;
    uint32_t       id = 0;
    void           applyCTM();
};

struct {
    SP<CCWlRegistry>                  pRegistry;
    SP<CCHyprlandCtmControlManagerV1> pCTMMgr;
    wl_display*                       wlDisplay = nullptr;
    std::vector<SP<SOutput>>          outputs;
    bool                              initialized = false;
    Mat3x3             ctm;
} state;

// Blocking queue used to communicate between threads
template <typename T> class Queue {
 public:
  Queue(): semaphore(0) {}

  void push(const T& t) {
    std::lock_guard<std::mutex> guard(lock);
    queue.push(t);
    semaphore.release();
  };

  T pop() {
    semaphore.acquire();
    std::lock_guard<std::mutex> guard(lock);
    T t = queue.front();
    queue.pop();
    return t;
  }

 private:
  std::queue<T> queue;
  std::mutex lock;
  std::counting_semaphore<> semaphore;
};

struct Transition {
  int hour;
  int minute;
  int kelvin;
  bool identity;

  int SecondOfDay() const {
    return hour * 60 * 60 + minute * 60;
  }

  string Kelvin() const {
    return identity ? "identity" : std::to_string(kelvin) + "K";
  }

  Mat3x3 Matrix() const {
    return identity ? Mat3x3::identity() : matrixForKelvin(kelvin);
  }
};

Transition CreateTransition(const string& arg, int kelvin, bool identity) {
  return Transition{
    .hour = stoi(arg.substr(0, 2)),
    .minute = stoi(arg.substr(2, 4)),
    .kelvin = kelvin,
    .identity = identity,
  };
}

// Parses a transition from the command-line
Transition ParseTransition(const string& arg) {
  // Special case for the identity matrix (suffixed with :i or :identity)
  // Otherwise the only acceptable format is <4 digits>:<4 digits>
  if (arg.ends_with(":i") && arg.size() == 6) {
    return CreateTransition(arg, kIdentityKelvin, true);
  } else if (arg.ends_with(":identity") && arg.size() == 13) {
    return CreateTransition(arg, kIdentityKelvin, true);
  } else if (arg.size() != 9 || arg[4] != ':') {
    throw std::runtime_error(std::format("invalid argument: {}", arg));
  }
  const int kelvin = stoi(arg.substr(5, 9));
  return CreateTransition(arg, kelvin, false);
}

// Finds an iterator corresponding to the upcoming transition & how long we should wait for it (in seconds)
// The transitions vector cannot be empty.
pair<vector<Transition>::const_iterator, int> NextTransitionIt(const vector<Transition>& transitions) {
  struct tm now;
  time_t ts = time(NULL);
  localtime_r(&ts, &now);
  auto it = std::find_if(transitions.begin(), transitions.end(), [&ts, &now](const Transition& t) {
    return t.hour > now.tm_hour || (t.hour == now.tm_hour && t.minute > now.tm_min);
  });
  // Not found means we're after the last one -> return the first one, but it's tomorrow
  const bool tomorrow = it == transitions.end();
  if (tomorrow) {
    it = transitions.begin();
  }
  const int second_of_day = it->SecondOfDay() + (tomorrow ? 24 * 60 * 60 : 0);
  const int current_second = now.tm_hour * 60 * 60 + now.tm_min * 60 + now.tm_sec;
  return pair<vector<Transition>::const_iterator, int>(it, second_of_day - current_second);
}

// Finds the previous transition
Transition PrevTransition(const vector<Transition>& transitions) {
  const auto next = NextTransitionIt(transitions).first;
  return next == transitions.begin() ? transitions.back() : *std::prev(next);
}

// Finds the upcoming transition, and how long we should wait for it (in seconds)
// Very similar to NextTransitionIt except a bit more convenient below.
pair<Transition, int> NextTransition(const vector<Transition>& transitions) {
  const auto next = NextTransitionIt(transitions);
  return pair<Transition, int>(*next.first, next.second);
}

void sigHandler(int sig) {
    if (state.pCTMMgr) // reset the CTM state...
        state.pCTMMgr.reset();

    Debug::log(NONE, "┣ Exiting on user interrupt\n╹");

    exit(0);
}

void SOutput::applyCTM() {
    auto arr = state.ctm.getMatrix();
    state.pCTMMgr->sendSetCtmForOutput(output->resource(), wl_fixed_from_double(arr[0]), wl_fixed_from_double(arr[1]), wl_fixed_from_double(arr[2]), wl_fixed_from_double(arr[3]),
                                       wl_fixed_from_double(arr[4]), wl_fixed_from_double(arr[5]), wl_fixed_from_double(arr[6]), wl_fixed_from_double(arr[7]),
                                       wl_fixed_from_double(arr[8]));
}

void writeFile(const string& file, int kelvin, int minTemp, int maxTemp) {
  if (!file.empty()) {
    try {
      const double percentage = 100.0 * (double)(kelvin - minTemp) / (double)(maxTemp - minTemp);
      ofstream f(file);
      f << std::format(R"({{"text":"{}K","tooltip":"Current temperature: {}K","class: "p{:0.0f}","percentage":{:0.2f}}})", kelvin, kelvin, percentage, percentage);
      f.close();
    } catch (std::exception& ex) {
      Debug::log(WARN, "✖ Couldn't write output file: {}", ex.what());
    }
  }
}

static void commitCTMs() {
    state.pCTMMgr->sendCommit();
}

static void printHelp() {
  Debug::log(NONE, "┣ Usage:");
  Debug::log(NONE, "┣ --transition        -t  →  Set the transition time / temperature in kelvin (e.g. 2100:4000)");
  Debug::log(NONE, "┣ --duration          -d  →  The duration (in seconds) to fade to the new temperature over");
  Debug::log(NONE, "┣ --file              -f  →  Filename to write the latest temperature update to");
  Debug::log(NONE, "┣ --help              -h  →  Print this info");
  Debug::log(NONE, "╹");
}

int main(int argc, char** argv, char** envp) {
    Debug::log(NONE, "┏ hyprsunset v{} ━━╸", HYPRSUNSET_VERSION);
    Debug::log(NONE, "┃");

    vector<Transition> transitions;
    int duration = 0;
    string file;
    for (int i = 1; i < argc; ++i) {
        if (argv[i] == std::string{"-t"} || argv[i] == std::string{"--transition"}) {
            if (i + 1 >= argc) {
              Debug::log(CRIT, "✖ No argument provided for {}", argv[i]);
              return 1;
            }
            try {
              transitions.push_back(ParseTransition(argv[i + 1]));
            } catch (std::exception& ex) {
              Debug::log(CRIT, "✖ Transition {} is not valid: {}", argv[i + 1], ex.what());
              return 1;
            }
            ++i;
        } else if (argv[i] == std::string{"-d"} || argv[i] == std::string{"--duration"}) {
            if (i + 1 >= argc) {
              Debug::log(CRIT, "✖ No argument provided for {}", argv[i]);
              return 1;
            }
            try {
              duration = stoi(argv[i + 1]);
            } catch (std::exception& ex) {
              Debug::log(CRIT, "✖ Duration {} is not valid", argv[i + 1]);
              return 1;
            }
            ++i;
        } else if (argv[i] == std::string{"-f"} || argv[i] == std::string{"--file"}) {
            if (i + 1 >= argc) {
              Debug::log(CRIT, "✖ No argument provided for {}", argv[i]);
              return 1;
            }
            file = argv[i + 1];
            ++i;
        } else if (argv[i] == std::string{"-h"} || argv[i] == std::string{"--help"}) {
            printHelp();
            return 0;
        } else {
            Debug::log(CRIT, "✖ Argument not recognized: {}", argv[i]);
            printHelp();
            return 1;
        }
    }
    if (transitions.empty()) {
      Debug::log(CRIT, "✖ Required argument -t / --transition not passed at least once");
      return 1;
    }
    Debug::log(INFO, "┣ Transitions loaded:");
    for (const auto& t: transitions) {
      Debug::log(INFO, "┣   {:02}:{:02}: {} {}", t.hour, t.minute, t.Kelvin(), t.Matrix().toString());
    }
    auto compareKelvin = [] (const Transition& a, const Transition& b) {
      return a.kelvin < b.kelvin;
    };
    const int minTemp = std::min_element(transitions.begin(), transitions.end(), compareKelvin)->kelvin;
    const int maxTemp = std::max_element(transitions.begin(), transitions.end(), compareKelvin)->kelvin;
    auto prevTransition = PrevTransition(transitions);
    Debug::log(INFO, "┣ Current state: {:02}:{:02}: {}", prevTransition.hour, prevTransition.minute, prevTransition.Kelvin());

    // set this as the matrix
    state.ctm = prevTransition.Matrix();

    Debug::log(NONE, "┣ Calculated the CTM to be {}", state.ctm.toString());
    Debug::log(NONE, "┃");

    // connect to the wayland server
    if (const auto SERVER = getenv("XDG_CURRENT_DESKTOP"); SERVER)
        Debug::log(NONE, "┣ Running on {}", SERVER);

    state.wlDisplay = wl_display_connect(nullptr);

    if (!state.wlDisplay) {
        Debug::log(NONE, "✖ Couldn't connect to a wayland compositor");
        return 1;
    }

    signal(SIGTERM, sigHandler);

    state.pRegistry = makeShared<CCWlRegistry>((wl_proxy*)wl_display_get_registry(state.wlDisplay));
    state.pRegistry->setGlobal([](CCWlRegistry* r, uint32_t name, const char* interface, uint32_t version) {
        const std::string IFACE = interface;

        if (IFACE == hyprland_ctm_control_manager_v1_interface.name) {
            Debug::log(NONE, "┣ Found hyprland-ctm-control-v1 supported with version {}, binding to v1", version);
            state.pCTMMgr = makeShared<CCHyprlandCtmControlManagerV1>(
                (wl_proxy*)wl_registry_bind((wl_registry*)state.pRegistry->resource(), name, &hyprland_ctm_control_manager_v1_interface, 1));
        } else if (IFACE == wl_output_interface.name) {

            if (std::find_if(state.outputs.begin(), state.outputs.end(), [name](const auto& el) { return el->id == name; }) != state.outputs.end())
                return;

            Debug::log(NONE, "┣ Found new output with ID {}, binding", name);
            auto o = state.outputs.emplace_back(
                makeShared<SOutput>(makeShared<CCWlOutput>((wl_proxy*)wl_registry_bind((wl_registry*)state.pRegistry->resource(), name, &wl_output_interface, 1)), name));

            if (state.initialized) {
                Debug::log(NONE, "┣ already initialized, applying CTM instantly", name);
                o->applyCTM();
                commitCTMs();
            }
        }
    });

    wl_display_roundtrip(state.wlDisplay);

    if (!state.pCTMMgr) {
        Debug::log(CRIT, "✖ Compositor doesn't support hyprland-ctm-control-v1, are you running on Hyprland?");
        return 1;
    }

    Debug::log(NONE, "┣ Found {} outputs, applying CTMs", state.outputs.size());
    state.initialized = true;
    Queue<Transition> queue;
    queue.push(prevTransition);

    std::thread thread([&] {
      while (true) {
        auto [transition, wait] = NextTransition(transitions);
        Debug::log(INFO, "┣ Waiting {}s for next transition (at {:02}:{:02})", wait, transition.hour, transition.minute);
        // Some care is needed here to deal with the machine suspending; sleep(3) will oversleep in those cases.
        struct timespec ts{.tv_sec = wait, .tv_nsec = 0};
        while (clock_nanosleep(CLOCK_BOOTTIME, 0, &ts, &ts) == EINTR);

        if (duration > 0) {
          clock_gettime(CLOCK_BOOTTIME, &ts);
          Debug::log(INFO, "┣ Beginning transition to {}: {}", transition.kelvin, transition.Matrix().toString());
          int lastKelvin = prevTransition.kelvin;
          for (int i = 0; i < duration; ++i) {
            double proportion = (double)i / double(duration);
            int kelvin = (int)(proportion * (transition.kelvin - prevTransition.kelvin)) + prevTransition.kelvin;
            if (kelvin != lastKelvin) {
              queue.push(Transition{.kelvin = kelvin});
              lastKelvin = kelvin;
            }
            ts.tv_sec++;
            clock_nanosleep(CLOCK_BOOTTIME, TIMER_ABSTIME, &ts, NULL);
          }
        }
        Debug::log(INFO, "┣ New CTM of {}: {}", transition.kelvin, transition.Matrix().toString());
        queue.push(transition);
        prevTransition = transition;
      }
    });

    while (true) {
      wl_display_flush(state.wlDisplay);
      if (wl_display_prepare_read(state.wlDisplay) == 0) {
        wl_display_read_events(state.wlDisplay);
        wl_display_dispatch_pending(state.wlDisplay);
      } else {
        wl_display_dispatch(state.wlDisplay);
      }
      const Transition t = queue.pop();
      state.ctm = t.Matrix();
      for (auto& o : state.outputs) {
        o->applyCTM();
      }
      commitCTMs();
      writeFile(file, t.kelvin, minTemp, maxTemp);
    }

    return 0;
}
