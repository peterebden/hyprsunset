#include <iostream>
#include <cmath>
#include <algorithm>
#include <format>
#include <mutex>
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

using std::pair;
using std::vector;
using std::string;
using std::stoi;

// kindly borrowed from https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
static Mat3x3 matrixForKelvin(unsigned long long temp) {
    float r = 1.F, g = 1.F, b = 1.F;

    temp /= 100;

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

    Mat3x3 CTM() const {
      std::lock_guard<std::mutex> guard(lock);
      return ctm;
    }

    void SetCTM(const Mat3x3& matrix) {
      std::lock_guard<std::mutex> guard(lock);
      ctm = matrix;
    }

private:
    Mat3x3             ctm;
    mutable std::mutex lock;
} state;

struct Transition {
  int hour;
  int minute;
  int kelvin;
  Mat3x3 matrix;

  int SecondOfDay() const {
    return hour * 60 * 60 + minute * 60;
  }
};

// Parses a transition from the command-line
Transition ParseTransition(const string& arg) {
  // Only acceptable format is <4 digits>:<4 digits>
  if (arg.size() != 9 || arg[4] != ':') {
    throw std::runtime_error(std::format("invalid argument: {}", arg));
  }
  const int kelvin = stoi(arg.substr(5, 9));
  return Transition{
    .hour = stoi(arg.substr(0, 2)),
    .minute = stoi(arg.substr(2, 4)),
    .kelvin = kelvin,
    .matrix = matrixForKelvin(kelvin),
  };
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
    auto arr = state.CTM().getMatrix();
    state.pCTMMgr->sendSetCtmForOutput(output->resource(), wl_fixed_from_double(arr[0]), wl_fixed_from_double(arr[1]), wl_fixed_from_double(arr[2]), wl_fixed_from_double(arr[3]),
                                       wl_fixed_from_double(arr[4]), wl_fixed_from_double(arr[5]), wl_fixed_from_double(arr[6]), wl_fixed_from_double(arr[7]),
                                       wl_fixed_from_double(arr[8]));
}

static void commitCTMs() {
    state.pCTMMgr->sendCommit();
}

static void printHelp() {
  Debug::log(NONE, "┣ Usage: hyprsunset <time1>:<temperature1> [<time2>:<temperature2> ...]");
  Debug::log(NONE, "┣ For example: hyprsunset 0900:6000 2100:4000");
  Debug::log(NONE, "╹");
}

int main(int argc, char** argv, char** envp) {
    Debug::log(NONE, "┏ hyprsunset v{} ━━╸", HYPRSUNSET_VERSION);
    Debug::log(NONE, "┃");

    const vector<string> args(argv + 1, argv + argc);
    if (std::find(args.begin(), args.end(), "--help") != args.end() || std::find(args.begin(), args.end(), "-h") != args.end() || args.empty()) {
      printHelp();
      return 0;
    }
    vector<Transition> transitions;
    try {
      std::transform(args.begin(), args.end(), std::back_inserter(transitions), ParseTransition);
    } catch (std::exception& ex) {
      Debug::log(CRIT, "%s", ex.what());
      return 1;
    }
    Debug::log(INFO, "┣ Transitions loaded:");
    for (const auto& t: transitions) {
      Debug::log(INFO, "┣   {:02}:{:02}: {}K {}", t.hour, t.minute, t.kelvin, t.matrix.toString());
    }
    auto t = PrevTransition(transitions);
    Debug::log(INFO, "┣ Current state: {:02}:{:02}: {}K", t.hour, t.minute, t.kelvin);

    // set this as the matrix
    state.SetCTM(t.matrix);

    Debug::log(NONE, "┣ Calculated the CTM to be {}", state.CTM().toString());
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
        Debug::log(NONE, "✖ Compositor doesn't support hyprland-ctm-control-v1, are you running on Hyprland?");
        return 1;
    }

    auto applyCTMs = [] {
        for (auto& o : state.outputs) {
          o->applyCTM();
        }
        commitCTMs();
    };

    Debug::log(NONE, "┣ Found {} outputs, applying CTMs", state.outputs.size());
    applyCTMs();
    state.initialized = true;

    std::thread thread([&] {
      while (true) {
        auto [transition, wait] = NextTransition(transitions);
        Debug::log(INFO, "┣ Waiting {}s for next transition (at {:02}:{:02})", wait, transition.hour, transition.minute);
        sleep(wait);
        Debug::log(INFO, "┣ Applying CTM of {}K: {}", transition.kelvin, transition.matrix.toString());
        state.SetCTM(t.matrix);
        applyCTMs();
      }
    });

    while (wl_display_dispatch(state.wlDisplay) != -1) {
      ;
    }

    return 0;
}
