#!/usr/bin/env python3

import argparse
import datetime as dt
import importlib
import re
import textwrap
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


CTRLR_RE = re.compile(r"NVMe Controller at\s+(?P<bdf>[^\s]+)\s+\[(?P<id>[0-9a-fA-F]{4}:[0-9a-fA-F]{4})\]")


@dataclass
class ControllerReport:
    source_file: str
    name: str
    bdf: str
    pci_id: str
    attributes: Dict[str, str]


def _is_underline(line: str) -> bool:
    stripped = line.strip()
    return bool(stripped) and set(stripped) <= {"="}


def parse_nvme_info(path: Path) -> ControllerReport:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()

    bdf = "unknown"
    pci_id = "unknown"
    name = path.stem

    for line in lines:
        m = CTRLR_RE.search(line)
        if m:
            bdf = m.group("bdf")
            pci_id = m.group("id").lower()
            name = f"{bdf} [{pci_id}]"
            break

    attrs: Dict[str, str] = {}
    section = "General"
    subsection_stack: List[Tuple[int, str]] = []

    i = 0
    while i < len(lines):
        raw = lines[i]
        stripped = raw.strip()

        if not stripped:
            i += 1
            continue

        if i + 1 < len(lines) and _is_underline(lines[i + 1]):
            section = stripped
            subsection_stack.clear()
            i += 2
            continue

        if _is_underline(stripped):
            i += 1
            continue

        indent = len(raw) - len(raw.lstrip(" "))

        if ":" in stripped:
            key, value = stripped.split(":", 1)
            key = key.strip()
            value = value.strip() if value.strip() else "<empty>"

            while subsection_stack and subsection_stack[-1][0] >= indent:
                subsection_stack.pop()

            path_parts = [section] + [label for _, label in subsection_stack] + [key]
            attr_path = " / ".join(path_parts)
            attrs[attr_path] = value
        else:
            while subsection_stack and subsection_stack[-1][0] >= indent:
                subsection_stack.pop()
            subsection_stack.append((indent, stripped))

        i += 1

    model_key = "Controller Capabilities/Features / Model Number"
    model = attrs.get(model_key, "")
    if model:
        name = f"{name} | {model}"

    return ControllerReport(
        source_file=str(path),
        name=name,
        bdf=bdf,
        pci_id=pci_id,
        attributes=attrs,
    )


def merge_reports(reports: List[ControllerReport]) -> Dict[str, Dict[str, str]]:
    merged: Dict[str, Dict[str, str]] = defaultdict(dict)
    for report in reports:
        for attr_path, value in report.attributes.items():
            merged[attr_path][report.name] = value
    return dict(merged)


def _status(values: List[str]) -> str:
    unique = set(values)
    if len(unique) == 1:
        return "SAME"
    return "DIFF"


def _controller_value_labels(reports: List[ControllerReport]) -> List[str]:
    labels: List[str] = []
    used: Dict[str, int] = defaultdict(int)

    for idx, report in enumerate(reports, start=1):
        label = f"Controller {idx}"
        if " | " in report.name:
            candidate = report.name.split(" | ", 1)[1].strip()
            if candidate:
                label = candidate

        used[label] += 1
        if used[label] > 1:
            label = f"{label} ({used[label]})"

        labels.append(label)

    return labels


def _build_grouped_attributes(
    reports: List[ControllerReport],
    merged: Dict[str, Dict[str, str]],
    only_diff: bool,
) -> Tuple[Dict[str, List[Tuple[str, str, Dict[str, str]]]], int, int]:
    controllers = [r.name for r in reports]
    grouped: Dict[str, List[Tuple[str, str, Dict[str, str]]]] = defaultdict(list)

    diff_count = 0
    rendered_count = 0

    for attr_path in sorted(merged.keys()):
        section = attr_path.split(" / ", 1)[0]
        rel = attr_path[len(section) + 3:] if attr_path.startswith(section + " / ") else attr_path
        values_by_controller = {c: merged[attr_path].get(c, "<missing>") for c in controllers}
        status = _status(list(values_by_controller.values()))

        if status == "DIFF":
            diff_count += 1
        if only_diff and status != "DIFF":
            continue

        rendered_count += 1
        grouped[section].append((rel, status, values_by_controller))

    return dict(grouped), rendered_count, diff_count


def render_txt_report(
    reports: List[ControllerReport],
    merged: Dict[str, Dict[str, str]],
    only_diff: bool = False,
) -> str:
    controller_keys = [r.name for r in reports]
    controller_labels = _controller_value_labels(reports)
    controller_col_width = max((len(c) for c in controller_labels), default=0)
    grouped, rendered_count, diff_count = _build_grouped_attributes(reports, merged, only_diff)

    out: List[str] = []
    out.append("Merged NVMe Context Report")
    out.append("=" * 26)
    out.append(f"Generated: {dt.datetime.now().isoformat(timespec='seconds')}")
    out.append("")
    out.append("Input files:")
    for r in reports:
        out.append(f"- {r.source_file}")
    out.append("")
    out.append("Controllers:")
    for idx, r in enumerate(reports, start=1):
        out.append(f"{idx}. {r.name}")
    out.append("")

    for section in sorted(grouped.keys()):
        section_lines: List[str] = []

        for rel, status, values_by_controller in grouped[section]:
            section_lines.append(f"* {rel} [{status}]")
            for key, label in zip(controller_keys, controller_labels):
                value = values_by_controller.get(key, "<missing>")
                section_lines.append(f"    - {label.ljust(controller_col_width)} : {value}")
            section_lines.append("")

        if not section_lines:
            continue

        out.append(section)
        out.append("-" * len(section))

        out.extend(section_lines)

    out.insert(0, f"Attributes rendered: {rendered_count}")
    out.insert(1, f"Attributes with DIFF: {diff_count}")
    out.insert(2, f"Mode: {'only-diff' if only_diff else 'all'}")
    out.insert(3, "")

    return "\n".join(out).rstrip() + "\n"


def render_pdf_report(
    reports: List[ControllerReport],
    merged: Dict[str, Dict[str, str]],
    output_path: Path,
    only_diff: bool = False,
) -> None:
    try:
        colors = importlib.import_module("reportlab.lib.colors")
        pagesizes = importlib.import_module("reportlab.lib.pagesizes")
        pdf_canvas = importlib.import_module("reportlab.pdfgen.canvas")
        A4 = pagesizes.A4
    except ImportError as exc:
        raise RuntimeError(
            "PDF output requires reportlab. Install it with: pip install reportlab"
        ) from exc

    controller_keys = [r.name for r in reports]
    controller_labels = _controller_value_labels(reports)
    controller_col_width = max((len(c) for c in controller_labels), default=0)
    grouped, rendered_count, diff_count = _build_grouped_attributes(reports, merged, only_diff)

    pdf = pdf_canvas.Canvas(str(output_path), pagesize=A4)
    page_w, page_h = A4
    left_margin = 36
    right_margin = 36
    top_margin = 36
    bottom_margin = 36
    max_chars = 125
    y = page_h - top_margin
    line_height = 12

    def write_line(text: str, color=colors.black, font="Helvetica", size=9, extra_space=0):
        nonlocal y
        wrapped = textwrap.wrap(text, width=max_chars) or [""]
        for line in wrapped:
            if y <= bottom_margin:
                pdf.showPage()
                y = page_h - top_margin
            pdf.setFont(font, size)
            pdf.setFillColor(color)
            pdf.drawString(left_margin, y, line)
            y -= line_height
        if extra_space:
            y -= extra_space

    write_line("Merged NVMe Context Report", font="Helvetica-Bold", size=13)
    write_line(f"Generated: {dt.datetime.now().isoformat(timespec='seconds')}")
    write_line(f"Mode: {'only-diff' if only_diff else 'all'}")
    write_line(f"Attributes rendered: {rendered_count}")
    write_line(f"Attributes with DIFF: {diff_count}", extra_space=4)

    write_line("Input files:", font="Helvetica-Bold")
    for r in reports:
        write_line(f"- {r.source_file}")
    write_line("", extra_space=2)

    write_line("Controllers:", font="Helvetica-Bold")
    for idx, r in enumerate(reports, start=1):
        write_line(f"{idx}. {r.name}")
    write_line("", extra_space=2)

    for section in sorted(grouped.keys()):
        write_line(section, font="Helvetica-Bold", size=11)

        for rel, status, values_by_controller in grouped[section]:
            status_color = colors.red if status == "DIFF" else colors.black
            write_line(f"* {rel} [{status}]", color=status_color, font="Helvetica-Bold")
            for key, label in zip(controller_keys, controller_labels):
                value = values_by_controller.get(key, "<missing>")
                aligned_line = f"    - {label.ljust(controller_col_width)} : {value}"
                write_line(aligned_line, font="Courier")
            write_line("")

    pdf.save()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Merge multiple SPDK NVMe info TXT outputs into one TXT comparison report."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        help="Input info_*.txt files (e.g. info_samsung.txt info_intel.txt info_skhynx.txt)",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="merged_nvme_info.txt",
        help="Output TXT file path (default: merged_nvme_info.txt)",
    )
    parser.add_argument(
        "--only-diff",
        action="store_true",
        help="Include only attributes that differ across controllers.",
    )
    parser.add_argument(
        "--pdf",
        nargs="?",
        const="merged_nvme_info.pdf",
        default=None,
        help="Also generate a PDF report. Optional path (default: merged_nvme_info.pdf).",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    input_paths = [Path(p) for p in args.inputs]
    missing = [str(p) for p in input_paths if not p.exists()]
    if missing:
        parser.error(f"Input file(s) not found: {', '.join(missing)}")

    reports = [parse_nvme_info(p) for p in input_paths]
    merged = merge_reports(reports)
    output_text = render_txt_report(reports, merged, only_diff=args.only_diff)

    output_path = Path(args.output)
    output_path.write_text(output_text, encoding="utf-8")
    print(f"Merged report written to: {output_path}")

    if args.pdf is not None:
        pdf_path = Path(args.pdf)
        render_pdf_report(reports, merged, pdf_path, only_diff=args.only_diff)
        print(f"PDF report written to: {pdf_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
