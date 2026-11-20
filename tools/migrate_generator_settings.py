#!/usr/bin/env python3
"""Migrate a saved flow graph across the renames and the setting moves of this repository.

1. The waveform frequency of gr::blocks::basic::SignalGenerator and
   gr::blocks::basic::FunctionGenerator moved off `frequency`, which is a reserved
   stream-tag key naming the stream's center frequency, and is now `tone_frequency`.
   A saved graph that names `frequency` on either block no longer reaches the
   waveform. Only those two blocks are touched: `frequency` on, say, a Soapy source
   is the center frequency and stays as it is.

2. Thirty-three families were registered under an alias that repeated a fixed
   template argument its name already stated, so the alias key carried that
   argument: gr::blocks::math::Add<float32, std::plus<float32>> where a file writes
   Add<float32>. Those alias keys now expand over the value type alone, so every
   argument after the first is dropped:

     gr::blocks::basic::SchmittTrigger                        gr::blocks::math::Add
     gr::blocks::basic::SchmittTriggerBasic                   gr::blocks::math::AddConst
     gr::blocks::basic::SchmittTriggerNoInterpolation         gr::blocks::math::Divide
     gr::blocks::basic::SchmittTriggerPolynomial              gr::blocks::math::DivideConst
     gr::blocks::basic::StreamFilter                          gr::blocks::math::Multiply
     gr::blocks::basic::StreamToDataSet                       gr::blocks::math::MultiplyConst
     gr::blocks::electrical::SinglePhasePowerFactorCalculator gr::blocks::math::Subtract
     gr::blocks::electrical::SinglePhasePowerMetrics          gr::blocks::math::SubtractConst
     gr::blocks::electrical::ThreePhasePowerFactorCalculator  gr::blocks::sdr::SoapyDualSink
     gr::blocks::electrical::ThreePhasePowerMetrics           gr::blocks::sdr::SoapyDualSource
     gr::blocks::electrical::ThreePhaseSystemUnbalanceCalculator gr::blocks::sdr::SoapyQuadSink
     gr::blocks::electrical::TwoPhaseSystemUnbalanceCalculator gr::blocks::sdr::SoapySink
     gr::blocks::filter::FrequencyEstimatorFrequencyDomainDecimating gr::blocks::sdr::SoapySource
     gr::blocks::filter::FrequencyEstimatorTimeDomainDecimating  gr::blocks::testing::ConsoleDebugSink
     gr::blocks::filter::IQDemodulator                        gr::blocks::testing::ImChartMonitor
     gr::blocks::testing::TagMonitor                          gr::blocks::testing::TagSink
     gr::blocks::testing::TagSource

   Nothing about the C++ types changed, so each block's primary key -- the
   expansion an alias resolves to -- is what it was: a graph naming the expansion
   needs no migration. Neither do gr::blocks::filter::BasicFilter,
   BasicDecimatingFilter or the four IirFilterDirectForm* names, which only gained
   keys beside their unchanged expansions.

3. The settings keep no contexts, and the loader refuses a graph file that holds
   `ctx_parameters`. gr::blocks::basic::FunctionGenerator holds the segments of
   its waveform in its `segments` setting, a map from a segment's name to the
   settings of the segment, and its `segment` setting selects one. Each context
   of the block's `ctx_parameters` becomes a segment of that name, and the block
   couples `segment` to the gr:context tag key, so a timing event that names a
   context selects that segment. A context with an empty name held the block's
   own values: its keys go to `parameters` where that does not name them
   already. The keys the block does not have are dropped, wherever they stand:
   signal_trigger, trigger_name, trigger_time, trigger_offset, trigger_meta_info
   and context. The `time` of a context is dropped as well, since a segment
   applies when it is selected. A timing event that names the segment in force
   again restarts nothing, where a context named again restarted its waveform;
   the block's `trigger` setting restarts it, and a graph that needs the restart
   couples `trigger` to a tag key whose tag carries true. The tool cannot tell
   which events meant a restart, so it adds no such coupling, and prints one line
   per migrated FunctionGenerator that says so.

4. gr::blocks::digital::AccessCodeCorrelator spaced its code's items by its
   `stride` setting, which is now `code_stride`: `stride` is the block-wide stride
   every block carries, read-only for this one. A saved graph that names `stride`
   on the correlator becomes one that names `code_stride`.

5. The enumerators a setting takes are spelled in lower case with underscores, as
   the standard library spells its own, and a host sends that spelling exactly:
   `bit_order: MsbFirst` becomes `bit_order: msb_first`, `window: BlackmanHarris`
   becomes `window: blackman_harris`. A value is respelled where its key is a
   setting an enumeration types, under any block; a value the table does not
   list stays as it is. Two enumerators were renamed, not respelled, since their
   spelling is a C++ keyword: the signal type `Const` is `constant`, and the
   Savitzky-Golay and SVD filters' boundary policy `Default` is `default_value`.
   The window `HannExp` becomes `hann`, the window it named.

Usage:
    python3 tools/migrate_generator_settings.py graph.grc              # report only
    python3 tools/migrate_generator_settings.py --in-place graph.grc   # rewrite

Self-test:
    python3 -m unittest discover -s tools -p 'migrate_*.py'
"""

import argparse
import re
import sys
import unittest

# each block whose key moved, with the key's old and new name
KEY_RENAMES = {
    "gr::blocks::basic::SignalGenerator": ("frequency", "tone_frequency"),
    "gr::blocks::basic::FunctionGenerator": ("frequency", "tone_frequency"),
    "gr::blocks::digital::AccessCodeCorrelator": ("stride", "code_stride"),
}

# each setting whose enumeration was respelled, with its enumerators' old and new names
ENUMERATOR_RENAMES = {
    "algorithm": {"Lms": "lms", "Nlms": "nlms", "Cma": "cma"},
    "alignment": {"Centred": "centered", "Causal": "causal"},
    "annotated_test_enum_setting": {
        "TEST_STATE1": "test_state1",
        "TEST_STATE2": "test_state2",
        "TEST_STATE3": "test_state3",
    },
    "bit_order": {"MsbFirst": "msb_first", "LsbFirst": "lsb_first"},
    "boundary_policy": {
        "Default": "default_value",
        "ZeroOrderHold": "zero_order_hold",
        "Reflect": "reflect",
        "Replicate": "replicate",
    },
    "burst_taper_type": {
        "None": "none",
        "Linear": "linear",
        "RaisedCosine": "raised_cosine",
        "Tukey": "tukey",
        "Gaussian": "gaussian",
        "Mushroom": "mushroom",
        "MushroomSine": "mushroom_sine",
    },
    "byte_order": {"Big": "big", "Little": "little"},
    "coding": {"Differential": "differential", "Nrzi": "nrzi"},
    "convention": {"Ieee8023": "ieee802_3", "GeThomas": "ge_thomas"},
    "count_covers": {
        "Payload": "payload",
        "PayloadAndCheck": "payload_and_check",
        "WholeFrame": "whole_frame",
    },
    "crc_byte_order": {"Big": "big", "Little": "little"},
    "derivative_method": {
        "SymmetricDifference": "symmetric_difference",
        "SavitzkyGolay5": "savitzky_golay5",
        "SavitzkyGolay7": "savitzky_golay7",
    },
    "detector": {
        "MuellerMuller": "mueller_muller",
        "ModifiedMuellerMuller": "modified_mueller_muller",
        "ZeroCrossing": "zero_crossing",
        "Gardner": "gardner",
        "EarlyLate": "early_late",
        "SignalTimesSlopeMl": "signal_times_slope_ml",
        "SignumTimesSlopeMl": "signum_times_slope_ml",
    },
    "direction": {"Apply": "apply", "Correct": "correct"},
    "drift_correction": {
        "None": "none",
        "Linear": "linear",
        "Cubic": "cubic",
        "AdaptiveResampling": "adaptive_resampling",
    },
    "filter_response": {
        "LOWPASS": "lowpass",
        "HIGHPASS": "highpass",
        "BANDPASS": "bandpass",
        "BANDSTOP": "bandstop",
    },
    "filter_type": {"FIR": "fir", "IIR": "iir"},
    "fir_design_method": {
        "None": "none",
        "Rectangular": "rectangular",
        "Hamming": "hamming",
        "Hann": "hann",
        "HannExp": "hann",
        "Blackman": "blackman",
        "Nuttall": "nuttall",
        "BlackmanHarris": "blackman_harris",
        "BlackmanNuttall": "blackman_nuttall",
        "FlatTop": "flat_top",
        "Exponential": "exponential",
        "Kaiser": "kaiser",
        "Bartlett": "bartlett",
        "Welch": "welch",
        "Parzen": "parzen",
        "Tukey": "tukey",
        "Gaussian": "gaussian",
    },
    "flags_position": {"High": "high", "Low": "low"},
    "frame_open": {
        "Delimiter": "delimiter",
        "Start": "start",
        "Immediate": "immediate",
    },
    "iir_design_method": {
        "BUTTERWORTH": "butterworth",
        "BESSEL": "bessel",
        "CHEBYSHEV1": "chebyshev1",
        "CHEBYSHEV2": "chebyshev2",
    },
    "input": {"Differential": "differential", "Chips": "chips"},
    "input_bit_order": {"MsbFirst": "msb_first", "LsbFirst": "lsb_first"},
    "kind": {
        "Complex": "complex",
        "Envelope": "envelope",
        "Block": "block",
        "Convolutional": "convolutional",
        "Permutation": "permutation",
    },
    "model": {"Rapp": "rapp", "Saleh": "saleh"},
    "normalization": {
        "Biased": "biased",
        "Unbiased": "unbiased",
        "Power": "power",
        "Amplitude": "amplitude",
        "None": "none",
    },
    "output_bit_order": {"MsbFirst": "msb_first", "LsbFirst": "lsb_first"},
    "payload_bit_order": {"MsbFirst": "msb_first", "LsbFirst": "lsb_first"},
    "phase_unit": {"Radians": "radians", "Degrees": "degrees"},
    "pmt_wire_format": {"GR3": "gr3", "GR4_YAML_V1": "gr4_yaml_v1"},
    "pulse": {"Rect": "rect", "RaisedCosine": "raised_cosine", "Gaussian": "gaussian"},
    "replacement": {"Interpolate": "interpolate", "Hold": "hold", "Zero": "zero"},
    "signal_type": {
        "Const": "constant",
        "LinearRamp": "linear_ramp",
        "ParabolicRamp": "parabolic_ramp",
        "CubicSpline": "cubic_spline",
        "ImpulseResponse": "impulse_response",
        "UniformNoise": "uniform_noise",
        "TriangularNoise": "triangular_noise",
        "GaussianNoise": "gaussian_noise",
        "Sin": "sin",
        "Cos": "cos",
        "FastSin": "fast_sin",
        "FastCos": "fast_cos",
        "Square": "square",
        "Saw": "saw",
        "Triangle": "triangle",
    },
    "soft_algorithm": {"MaxLog": "max_log", "Exact": "exact"},
    "termination": {"Terminated": "terminated", "Open": "open"},
    "test_enum_setting": {
        "TEST_STATE1": "test_state1",
        "TEST_STATE2": "test_state2",
        "TEST_STATE3": "test_state3",
    },
    "transparency": {
        "None": "none",
        "BitStuffing": "bit_stuffing",
        "ByteEscape": "byte_escape",
    },
    "type": {"GET": "get", "SUBSCRIBE": "subscribe"},
    "window": {
        "None": "none",
        "Rectangular": "rectangular",
        "Hamming": "hamming",
        "Hann": "hann",
        "HannExp": "hann",
        "Blackman": "blackman",
        "Nuttall": "nuttall",
        "BlackmanHarris": "blackman_harris",
        "BlackmanNuttall": "blackman_nuttall",
        "FlatTop": "flat_top",
        "Exponential": "exponential",
        "Kaiser": "kaiser",
        "Bartlett": "bartlett",
        "Welch": "welch",
        "Parzen": "parzen",
        "Tukey": "tukey",
        "Gaussian": "gaussian",
    },
}

ALIAS_FAMILIES = (
    "gr::blocks::basic::SchmittTrigger",
    "gr::blocks::basic::SchmittTriggerBasic",
    "gr::blocks::basic::SchmittTriggerNoInterpolation",
    "gr::blocks::basic::SchmittTriggerPolynomial",
    "gr::blocks::basic::StreamFilter",
    "gr::blocks::basic::StreamToDataSet",
    "gr::blocks::electrical::SinglePhasePowerFactorCalculator",
    "gr::blocks::electrical::SinglePhasePowerMetrics",
    "gr::blocks::electrical::ThreePhasePowerFactorCalculator",
    "gr::blocks::electrical::ThreePhasePowerMetrics",
    "gr::blocks::electrical::ThreePhaseSystemUnbalanceCalculator",
    "gr::blocks::electrical::TwoPhaseSystemUnbalanceCalculator",
    "gr::blocks::filter::FrequencyEstimatorFrequencyDomainDecimating",
    "gr::blocks::filter::FrequencyEstimatorTimeDomainDecimating",
    "gr::blocks::filter::IQDemodulator",
    "gr::blocks::math::Add",
    "gr::blocks::math::AddConst",
    "gr::blocks::math::Divide",
    "gr::blocks::math::DivideConst",
    "gr::blocks::math::Multiply",
    "gr::blocks::math::MultiplyConst",
    "gr::blocks::math::Subtract",
    "gr::blocks::math::SubtractConst",
    "gr::blocks::sdr::SoapyDualSink",
    "gr::blocks::sdr::SoapyDualSource",
    "gr::blocks::sdr::SoapyQuadSink",
    "gr::blocks::sdr::SoapySink",
    "gr::blocks::sdr::SoapySource",
    "gr::blocks::testing::ConsoleDebugSink",
    "gr::blocks::testing::ImChartMonitor",
    "gr::blocks::testing::TagMonitor",
    "gr::blocks::testing::TagSink",
    "gr::blocks::testing::TagSource",
)

_ID_LINE = re.compile(
    r"^(?P<lead>\s*(?:-\s+)?)id:(?P<gap>\s*)(?P<value>.*?)(?P<trail>\s*)$"
)
_RENAMED_LINE = re.compile(r"^(?P<indent>\s*)(?P<key>[A-Za-z_]+):(?P<rest>.*)$")


def split_top_level(arguments):
    """Split a template argument list on the commas that are not inside brackets."""
    parts, depth, current = [], 0, ""
    for char in arguments:
        if char in "<([":
            depth += 1
        elif char in ">)]":
            depth -= 1
        if char == "," and depth == 0:
            parts.append(current)
            current = ""
            continue
        current += char
    parts.append(current)
    return [part.strip() for part in parts]


def base_name(block_id):
    """The block id with its template argument list removed."""
    return block_id.split("<", 1)[0].strip()


def migrate_id(block_id):
    """The migrated block id, or None when the id needs no migration."""
    stripped = block_id.strip().strip("'\"")
    if (
        base_name(stripped) not in ALIAS_FAMILIES
        or "<" not in stripped
        or not stripped.endswith(">")
    ):
        return None
    name, _, arguments = stripped.partition("<")
    parts = split_top_level(arguments[:-1])
    if len(parts) < 2:
        return None
    return "{}<{}>".format(name, parts[0])


def migrate(text):
    """Return the migrated text and a list of the changes made, as (line number, description)."""
    lines = text.splitlines(keepends=True)
    changes = []
    item_indent = None  # indentation of the current item of a block in KEY_RENAMES, None when outside one
    renamed = None  # the (old, new) key of that block

    for number, line in enumerate(lines, start=1):
        body = line.rstrip("\r\n")
        indent = len(body) - len(body.lstrip())

        match = _ID_LINE.match(body)
        if match:
            block_id = match.group("value")
            renamed = KEY_RENAMES.get(base_name(block_id.strip().strip("'\"")))
            item_indent = indent if renamed is not None else None
            migrated = migrate_id(block_id)
            if migrated is not None:
                lines[number - 1] = "{}id:{}{}{}\n".format(
                    match.group("lead"),
                    match.group("gap"),
                    migrated,
                    match.group("trail"),
                )
                changes.append(
                    (number, "id {} -> {}".format(block_id.strip(), migrated))
                )
            continue

        if item_indent is not None and body.strip() and indent <= item_indent:
            item_indent = None  # the block's own lines are indented past its item

        if item_indent is not None:
            key = _RENAMED_LINE.match(body)
            if key and key.group("key") == renamed[0]:
                lines[number - 1] = "{}{}:{}\n".format(
                    key.group("indent"), renamed[1], key.group("rest")
                )
                changes.append((number, "{} -> {}".format(*renamed)))

    return "".join(lines), changes


FUNCTION_GENERATOR = "gr::blocks::basic::FunctionGenerator"
DROPPED_GENERATOR_KEYS = (
    "signal_trigger",
    "trigger_name",
    "trigger_time",
    "trigger_offset",
    "trigger_meta_info",
    "context",
)
SEGMENT_COUPLING = "segment: gr:context"
RESTART_NOTE = (
    "a context named twice in a row restarts the waveform only through a coupling to `trigger`; "
    "add one where a graph relies on the restart"
)
_PLAIN_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")
_KEY_LINE = re.compile(
    r"^(?P<indent>\s*)(?P<dash>-\s+)?(?P<key>[^\s:#][^:#]*?):(?:\s+(?P<value>.*?))?\s*$"
)


def _indent(line):
    return len(line) - len(line.lstrip())


def _unquote(text):
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "'\"":
        return text[1:-1]
    return text


def _segment_name(name):
    return name if _PLAIN_NAME.match(name) else '"{}"'.format(name.replace('"', '\\"'))


def _block_end(lines, start, item_indent):
    """The index past the last line of the list item that starts at `start`."""
    end = start + 1
    while end < len(lines):
        body = lines[end].rstrip("\r\n")
        if body.strip() and _indent(body) <= item_indent:
            break
        end += 1
    return end


def _section(lines, start, end, key_indent, key):
    """The [first, past) range of the mapping entry `key` at `key_indent` within [start, end), or None."""
    for index in range(start, end):
        body = lines[index].rstrip("\r\n")
        match = _KEY_LINE.match(body)
        if (
            match
            and not match.group("dash")
            and len(match.group("indent")) == key_indent
            and match.group("key") == key
        ):
            past = index + 1
            while past < end:
                inner = lines[past].rstrip("\r\n")
                if inner.strip() and _indent(inner) <= key_indent:
                    break
                past += 1
            return index, past
    return None


def _entries(lines, indent):
    """The (key, lines) of each entry at `indent` in `lines`, each entry's lines carrying its own nested lines."""
    entries = []
    for line in lines:
        body = line.rstrip("\r\n")
        if not body.strip():
            continue
        match = _KEY_LINE.match(body)
        if _indent(body) == indent and match and not match.group("dash"):
            entries.append((match.group("key"), [body]))
        elif entries:
            entries[-1][1].append(body)
    return entries


def _reindent(entry_lines, old_indent, new_indent):
    return [
        " " * new_indent + line[old_indent:] if line.strip() else line
        for line in entry_lines
    ]


def _contexts(lines):
    """Each (name, entry lines, entry indent) of a ctx_parameters list given as its lines after the key."""
    contexts = []
    items = []
    for line in lines:
        body = line.rstrip("\r\n")
        if body.lstrip().startswith("- "):
            items.append([body])
        elif items:
            items[-1].append(body)
    for item in items:
        dash = item[0].index("-")
        key_indent = (
            dash + len(item[0][dash + 1 :]) - len(item[0][dash + 1 :].lstrip()) + 1
        )
        flat = [" " * key_indent + item[0][key_indent:]] + item[1:]
        name = ""
        for key, entry in _entries(flat, key_indent):
            if key == "context":
                name = _unquote(entry[0].split(":", 1)[1])
        parameters = _section(flat, 0, len(flat), key_indent, "parameters")
        entries, entry_indent = [], key_indent + 2
        if parameters is not None:
            nested = [
                line for line in flat[parameters[0] + 1 : parameters[1]] if line.strip()
            ]
            if nested:
                entry_indent = _indent(nested[0])
                entries = _entries(nested, entry_indent)
        contexts.append((name, entries, entry_indent))
    return contexts


def migrate_contexts(text):
    """Return the text with each FunctionGenerator's ctx_parameters moved into segments, the changes made, and a note
    per migrated block."""
    lines = text.splitlines()
    changes = []
    notes = []
    index = 0
    while index < len(lines):
        match = _ID_LINE.match(lines[index])
        if not match or base_name(_unquote(match.group("value"))) != FUNCTION_GENERATOR:
            index += 1
            continue
        item_indent = _indent(lines[index])
        key_indent = len(match.group("lead"))
        end = _block_end(lines, index, item_indent)
        block = lines[index:end]
        contexts_range = _section(block, 1, len(block), key_indent, "ctx_parameters")
        parameters_range = _section(block, 1, len(block), key_indent, "parameters")

        own = []
        dropped = False
        if parameters_range is not None:
            nested = [
                line
                for line in block[parameters_range[0] + 1 : parameters_range[1]]
                if line.strip()
            ]
            entry_indent = _indent(nested[0]) if nested else key_indent + 2
            for key, entry in _entries(nested, entry_indent):
                if key in DROPPED_GENERATOR_KEYS:
                    changes.append((index + 1, "parameters: {} dropped".format(key)))
                    dropped = True
                else:
                    own.append((key, _reindent(entry, entry_indent, key_indent + 2)))
        if contexts_range is None and not dropped:
            index = end
            continue

        segments = []
        if contexts_range is not None:
            for name, entries, entry_indent in _contexts(
                block[contexts_range[0] + 1 : contexts_range[1]]
            ):
                kept = []
                for key, entry in entries:
                    if key in DROPPED_GENERATOR_KEYS:
                        changes.append(
                            (index + 1, "context {!r}: {} dropped".format(name, key))
                        )
                    else:
                        kept.append((key, entry, entry_indent))
                if name == "":
                    named = {key for key, _ in own}
                    for key, entry, old_indent in kept:
                        if key not in named:
                            own.append(
                                (key, _reindent(entry, old_indent, key_indent + 2))
                            )
                    changes.append((index + 1, "context '' -> parameters"))
                    continue
                body = []
                for key, entry, old_indent in kept:
                    body.extend(_reindent(entry, old_indent, key_indent + 6))
                segments.append((name, body))
                changes.append((index + 1, "context {!r} -> segment".format(name)))
            notes.append((index + 1, RESTART_NOTE))

        rebuilt = [block[0]]
        rest = [
            line
            for position, line in enumerate(block[1:], start=1)
            if not (
                parameters_range
                and parameters_range[0] <= position < parameters_range[1]
            )
            and not (
                contexts_range and contexts_range[0] <= position < contexts_range[1]
            )
        ]
        rebuilt.append(" " * key_indent + "parameters:")
        for _, entry in own:
            rebuilt.extend(entry)
        if segments:
            rebuilt.append(" " * (key_indent + 2) + "segments:")
            for name, body in segments:
                rebuilt.append(" " * (key_indent + 4) + _segment_name(name) + ":")
                rebuilt.extend(body if body else [" " * (key_indent + 6) + "{}"])
        couplings = _section(rest, 0, len(rest), key_indent, "tag_couplings")
        if segments and couplings is None:
            rest.append(" " * key_indent + "tag_couplings:")
            rest.append(" " * (key_indent + 2) + SEGMENT_COUPLING)
            changes.append((index + 1, "tag_couplings: segment -> gr:context"))
        elif segments and not any(
            line.strip().startswith("segment:")
            for line in rest[couplings[0] + 1 : couplings[1]]
        ):
            rest.insert(couplings[1], " " * (key_indent + 2) + SEGMENT_COUPLING)
            changes.append((index + 1, "tag_couplings: segment -> gr:context"))
        rebuilt.extend(rest)
        lines[index:end] = rebuilt
        index += len(rebuilt)

    trailing = "\n" if text.endswith("\n") else ""
    return "\n".join(lines) + trailing, changes, notes


def migrate_enumerators(text):
    """Return the text with each enumerator ENUMERATOR_RENAMES lists respelled, and the changes made."""
    lines = text.splitlines(keepends=True)
    changes = []
    for number, line in enumerate(lines, start=1):
        match = _KEY_LINE.match(line.rstrip("\r\n"))
        if not match or match.group("value") is None:
            continue
        renames = ENUMERATOR_RENAMES.get(match.group("key").strip())
        value = match.group("value")
        quote = (
            value[0]
            if value[:1] in ("'", '"') and value[-1:] == value[:1] and len(value) > 1
            else ""
        )
        bare = value[1:-1] if quote else value
        if renames is None or bare not in renames:
            continue
        ending = line[len(line.rstrip("\r\n")) :]
        lines[number - 1] = "{}{}{}: {}{}{}{}".format(
            match.group("indent"),
            match.group("dash") or "",
            match.group("key"),
            quote,
            renames[bare],
            quote,
            ending,
        )
        changes.append(
            (
                number,
                "{} {} -> {}".format(match.group("key").strip(), bare, renames[bare]),
            )
        )
    return "".join(lines), changes


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Migrate a saved flow graph across the generator setting and alias renames and the generator's segments."
    )
    parser.add_argument("graph", help="the saved graph file to migrate")
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="rewrite the file; without it the changes are only reported",
    )
    arguments = parser.parse_args(argv)

    with open(arguments.graph, "r", encoding="utf-8") as handle:
        original = handle.read()
    migrated, changes = migrate(original)
    migrated, context_changes, notes = migrate_contexts(migrated)
    changes += context_changes
    migrated, enumerator_changes = migrate_enumerators(migrated)
    changes += enumerator_changes

    for number, description in changes:
        print("{}:{}: {}".format(arguments.graph, number, description))
    for number, note in notes:
        print("{}:{}: note: {}".format(arguments.graph, number, note))
    if not changes:
        print("{}: nothing to migrate".format(arguments.graph))
        return 0
    if not arguments.in_place:
        print(
            "{}: {} change(s) not written; pass --in-place to rewrite".format(
                arguments.graph, len(changes)
            )
        )
        return 0
    with open(arguments.graph, "w", encoding="utf-8") as handle:
        handle.write(migrated)
    print("{}: {} change(s) written".format(arguments.graph, len(changes)))
    return 0


BEFORE = """\
blocks:
  - id: gr::blocks::basic::SignalGenerator<float32>
    parameters:
      name: tone
      frequency: 440
      sample_rate: 48000
  - id: gr::blocks::math::Add<float32, std::plus<float32>>
    parameters:
      name: sum
  - id: gr::blocks::sdr::SoapySource<complex<float32>, 1UZ>
    parameters:
      name: radio
      frequency: [107000000]
  - id: gr::blocks::math::MathOpMultiPortImpl<float32, std::plus<float32>>
    parameters:
      name: primary
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: ramp
    ctx_parameters:
      - context: one
        parameters:
          frequency: 10
connections:
  - [tone, 0, sum, 0]
"""

AFTER = """\
blocks:
  - id: gr::blocks::basic::SignalGenerator<float32>
    parameters:
      name: tone
      tone_frequency: 440
      sample_rate: 48000
  - id: gr::blocks::math::Add<float32>
    parameters:
      name: sum
  - id: gr::blocks::sdr::SoapySource<complex<float32>>
    parameters:
      name: radio
      frequency: [107000000]
  - id: gr::blocks::math::MathOpMultiPortImpl<float32, std::plus<float32>>
    parameters:
      name: primary
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: ramp
    ctx_parameters:
      - context: one
        parameters:
          tone_frequency: 10
connections:
  - [tone, 0, sum, 0]
"""

AFTER_SEGMENTS = AFTER.replace(
    """    ctx_parameters:
      - context: one
        parameters:
          tone_frequency: 10
""",
    """      segments:
        one:
          tone_frequency: 10
    tag_couplings:
      segment: gr:context
""",
)

GENERATOR_BEFORE = """\
blocks:
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: ramp
      sample_rate: 1000
      signal_trigger: CMD_BP_START
    ctx_parameters:
      - context: one
        time: 0
        parameters:
          signal_trigger: CMD_BP_START
          signal_type: Const
          start_value: 5
      - context: two
        time: 0
        parameters:
          signal_trigger: CMD_BP_START
          signal_type: LinearRamp
          start_value: 5
          final_value: 30
          duration: 0.2
"""

GENERATOR_AFTER = """\
blocks:
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: ramp
      sample_rate: 1000
      segments:
        one:
          signal_type: Const
          start_value: 5
        two:
          signal_type: LinearRamp
          start_value: 5
          final_value: 30
          duration: 0.2
    tag_couplings:
      segment: gr:context
"""

CORRELATOR_BEFORE = """\
blocks:
  - id: gr::blocks::digital::AccessCodeCorrelator<uint8>
    parameters:
      name: correlator
      stride: !!uint32 80
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: other
      stride: 2
"""


class MigrateTest(unittest.TestCase):
    def test_fixture_graph(self):
        migrated, changes = migrate(BEFORE)
        self.assertEqual(migrated, AFTER)
        self.assertEqual(len(changes), 4)

    def test_migration_is_idempotent(self):
        self.assertEqual(migrate(AFTER), (AFTER, []))

    def test_fixture_contexts_become_segments(self):
        migrated, changes, notes = migrate_contexts(AFTER)
        self.assertEqual(migrated, AFTER_SEGMENTS)
        self.assertEqual(len(changes), 2)
        self.assertEqual(notes, [(17, RESTART_NOTE)])

    def test_generator_contexts_become_segments(self):
        migrated, changes, notes = migrate_contexts(GENERATOR_BEFORE)
        self.assertEqual(migrated, GENERATOR_AFTER)
        self.assertEqual(notes, [(2, RESTART_NOTE)])
        self.assertIn((2, "context 'one' -> segment"), changes)
        self.assertIn((2, "tag_couplings: segment -> gr:context"), changes)

    def test_segment_migration_is_idempotent(self):
        self.assertEqual(migrate_contexts(GENERATOR_AFTER), (GENERATOR_AFTER, [], []))

    def test_empty_context_goes_to_parameters(self):
        before = GENERATOR_BEFORE.replace("context: one", "context: ''")
        migrated, _, _ = migrate_contexts(before)
        self.assertNotIn("        one:", migrated)
        self.assertIn(
            "      signal_type: Const\n      start_value: 5\n      segments:", migrated
        )

    def test_a_name_with_colons_is_quoted(self):
        before = GENERATOR_BEFORE.replace(
            "context: one", "context: FAIR.SELECTOR.C=1:S=1:P=1"
        )
        self.assertIn(
            '        "FAIR.SELECTOR.C=1:S=1:P=1":\n', migrate_contexts(before)[0]
        )

    def test_correlator_stride_becomes_code_stride(self):
        migrated, changes = migrate(CORRELATOR_BEFORE)
        self.assertIn("      code_stride: !!uint32 80\n", migrated)
        self.assertNotIn("stride: !!uint32", migrated.replace("code_stride", ""))
        self.assertEqual(changes, [(5, "stride -> code_stride")])
        self.assertIn(
            "      stride: 2\n", migrated, "another block's key of that name stays"
        )

    def test_center_frequency_is_left_alone(self):
        self.assertIn("      frequency: [107000000]", migrate(BEFORE)[0])

    def test_only_the_value_type_survives(self):
        self.assertEqual(
            migrate_id("gr::blocks::math::Add<float32, std::plus<float32>>"),
            "gr::blocks::math::Add<float32>",
        )
        self.assertEqual(
            migrate_id("gr::blocks::sdr::SoapySink<complex<float32>, 1UZ>"),
            "gr::blocks::sdr::SoapySink<complex<float32>>",
        )

    def test_enumerators_are_respelled(self):
        migrated, changes = migrate_enumerators(GENERATOR_AFTER)
        self.assertIn("          signal_type: constant\n", migrated)
        self.assertIn("          signal_type: linear_ramp\n", migrated)
        self.assertEqual(len(changes), 2)
        self.assertEqual(migrate_enumerators(migrated), (migrated, []))
        self.assertEqual(
            migrate_enumerators("      window: 'BlackmanHarris'\n")[0],
            "      window: 'blackman_harris'\n",
        )
        self.assertEqual(
            migrate_enumerators("      name: Hann\n")[0],
            "      name: Hann\n",
            "another key's value stays",
        )

    def test_unchanged_ids_report_nothing(self):
        self.assertIsNone(migrate_id("gr::blocks::math::Add<float32>"))
        self.assertIsNone(migrate_id("gr::blocks::filter::BasicFilterProto<float32>"))
        self.assertIsNone(
            migrate_id(
                "gr::blocks::math::MathOpMultiPortImpl<float32, std::plus<float32>>"
            )
        )


if __name__ == "__main__":
    sys.exit(main())
