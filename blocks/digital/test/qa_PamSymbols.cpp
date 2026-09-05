#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/digital/PamSymbols.hpp>

namespace {

using gr::blocks::digital::LevelTracker;
using gr::blocks::digital::PamSlicer;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
    return block;
}

/// A deterministic four-level symbol stream on the ±1/±3 grid, scaled and shifted.
[[nodiscard]] std::vector<float> symbols(std::size_t count, double scale, double shift, std::uint64_t seed) {
    std::uint64_t      state = seed;
    std::vector<float> out(count);
    for (float& v : out) {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        const int level = 2 * static_cast<int>(state & 3U) - 3; // -3 -1 +1 +3
        v               = static_cast<float>(static_cast<double>(level) * scale + shift);
    }
    return out;
}

[[nodiscard]] std::vector<float> run(LevelTracker<float>& block, std::span<const float> input, std::size_t chunk = 0UZ) {
    std::vector<float> out(input.size());
    const std::size_t  stride = chunk == 0UZ ? input.size() : chunk;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t until = std::min(base + stride, input.size());
        std::ignore             = block.processBulk(input.subspan(base, until - base), std::span<float>(out).subspan(base, until - base));
    }
    return out;
}

} // namespace

const boost::ut::suite<"PamSymbols"> pamSymbolsTests = [] {
    using namespace boost::ut;

    static_assert(!gr::HasConstProcessOneFunction<LevelTracker<float>>, "the tracker carries its loops");
    static_assert(gr::HasConstProcessOneFunction<PamSlicer<float>>, "the slicer is a pure table");

    "a clean stream settles spread at nominal and offset at zero"_test = [] {
        auto       block = make<LevelTracker<float>>();
        const auto in    = symbols(6000UZ, 1.0, 0.0, 0x9e3779b97f4a7c15ULL);
        const auto out   = run(block, in);
        expect(lt(std::abs(block.spread.value - 2.0), 0.02)) << std::format("spread {}", block.spread.value);
        expect(lt(std::abs(block.offset.value), 0.02)) << std::format("offset {}", block.offset.value);
        // Settled, the output IS the input: the grid was already nominal.
        double worst = 0.0;
        for (std::size_t i = 4000UZ; i < out.size(); ++i) {
            worst = std::max(worst, static_cast<double>(std::abs(out[i] - in[i])));
        }
        expect(lt(worst, 0.05)) << "the normalization is transparent on a nominal stream";
    };

    "a deviation 15 % high or low is measured, not just tolerated"_test = [] {
        for (const double scale : {1.15, 0.85}) {
            auto       block = make<LevelTracker<float>>();
            const auto in    = symbols(8000UZ, scale, 0.0, 0x2545F4914F6CDD1DULL);
            std::ignore      = run(block, in);
            expect(lt(std::abs(block.spread.value - 2.0 * scale), 0.05)) << std::format("scale {}: spread {}", scale, block.spread.value);
            expect(lt(std::abs(block.offset.value), 0.05));
        }
    };

    "a static shift up to four units is measured and removed, spacing undisturbed"_test = [] {
        for (const double shift : {-4.0, -1.25, 0.75, 4.0}) {
            auto       block = make<LevelTracker<float>>();
            const auto in    = symbols(8000UZ, 1.0, shift, 0x123456789abcdefULL);
            const auto out   = run(block, in);
            expect(lt(std::abs(block.offset.value - shift), 0.05)) << std::format("shift {}: offset {}", shift, block.offset.value);
            expect(lt(std::abs(block.spread.value - 2.0), 0.05)) << "a common shift does not disturb the spacing";
            // Settled, the output sits back on the nominal grid.
            double worst = 0.0;
            for (std::size_t i = 6000UZ; i < out.size(); ++i) {
                const float nearest = 2.0f * std::round(out[i] / 2.0f + 0.5f) - 1.0f; // nearest odd level
                worst               = std::max(worst, static_cast<double>(std::abs(out[i] - std::clamp(nearest, -3.0f, 3.0f))));
            }
            expect(lt(worst, 0.1)) << "settled symbols land on the grid";
        }
    };

    "the clamp bounds the spacing and lets it return"_test = [] {
        auto block  = make<LevelTracker<float>>();
        std::ignore = run(block, symbols(8000UZ, 2.0, 0.0, 0xdeadbeefULL)); // deviation doubled
        expect(lt(std::abs(block.spread.value - 2.4), 1e-6)) << "clamped at nominal * 1.2";
        std::ignore = run(block, symbols(8000UZ, 1.0, 0.0, 0xdeadbeefULL));
        expect(lt(std::abs(block.spread.value - 2.0), 0.05)) << "and returns once the stream does";
    };

    "the tracker does not depend on the chunking"_test = [] {
        const auto in        = symbols(5000UZ, 1.1, -0.6, 0xfeedfaceULL);
        auto       reference = make<LevelTracker<float>>();
        const auto want      = run(reference, in);
        for (const std::size_t chunk : {1UZ, 7UZ, 997UZ}) {
            auto block = make<LevelTracker<float>>();
            expect(that % (run(block, in, chunk) == want)) << std::format("chunk {}", chunk);
        }
    };

    "the slicer's decision table, thresholds and label maps"_test = [] {
        auto identity = make<PamSlicer<float>>();
        auto c4fm     = make<PamSlicer<float>>({{"labels", std::vector<gr::Size_t>{3U, 2U, 0U, 1U}}});

        struct Case {
            float        x;
            std::uint8_t rank;
        };
        // Ranks by region: the thresholds sit at -2, 0, +2 with ties toward the upper region,
        // and the outer regions are unbounded.
        for (const Case c : {Case{-1e30f, 0}, Case{-3.f, 0}, Case{-2.01f, 0}, Case{-2.f, 1}, Case{-1.f, 1}, Case{-0.01f, 1}, Case{0.f, 2}, Case{1.f, 2}, Case{1.99f, 2}, Case{2.f, 3}, Case{3.f, 3}, Case{1e30f, 3}}) {
            expect(eq(identity.processOne(c.x), c.rank)) << std::format("identity at {}", c.x);
            constexpr std::uint8_t kC4fm[4]{3U, 2U, 0U, 1U};
            expect(eq(c4fm.processOne(c.x), kC4fm[c.rank])) << std::format("c4fm at {}", c.x);
        }
    };

    "refusals fire by name"_test = [] {
        expect(throws([] { std::ignore = make<PamSlicer<float>>({{"labels", std::vector<gr::Size_t>{0U, 1U}}}); })) << "a label per level";
        expect(throws([] { std::ignore = make<PamSlicer<float>>({{"n_levels", gr::Size_t(3)}, {"labels", std::vector<gr::Size_t>{0U, 1U, 2U}}}); })) << "even M only";
        expect(throws([] { std::ignore = make<LevelTracker<float>>({{"n_levels", gr::Size_t(3)}}); })) << "even M only";
    };
};

int main() { /* not needed for UT */ }
