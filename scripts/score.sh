#!/usr/bin/env bash
# scripts/score.sh — recompute the Solidworks-tier total from PROGRESS.md
# Usage:
#   ./scripts/score.sh           # print total and update "Current score:" header
#   ./scripts/score.sh --check   # exit 1 if header doesn't match computed total

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROGRESS="$SCRIPT_DIR/../PROGRESS.md"

if [ ! -f "$PROGRESS" ]; then
    echo "ERROR: $PROGRESS not found" >&2
    exit 1
fi

# Extract the computed total by summing SW pts from table rows.
# Table rows look like:  | Some text | 15% | 99% | 14.85 |
# We skip:
#   - The header row (contains "SW weight")
#   - The total row (contains "Solidworks-tier total" or "OpenSCAD-tier")
#   - Rows that don't end with a bare number before the final |
COMPUTED=$(awk '
BEGIN { total = 0; rows = 0 }
/^\|/ {
    # Skip header row
    if ($0 ~ /SW weight/) next
    # Skip total/summary rows
    if ($0 ~ /Solidworks-tier total/) next
    if ($0 ~ /OpenSCAD-tier/) next

    # Extract last numeric cell: | ... | <number> |
    # The SW pts column is the last column before trailing |
    line = $0
    # Remove trailing whitespace and trailing |
    gsub(/[[:space:]]*\|[[:space:]]*$/, "", line)
    # Get the last field after the last |
    n = split(line, parts, "|")
    last = parts[n]
    # Trim whitespace
    gsub(/^[[:space:]]+|[[:space:]]+$/, "", last)
    # Must be a number (digits, optional decimal point)
    if (last ~ /^[0-9]+(\.[0-9]+)?$/) {
        total += last + 0
        rows++
    }
}
END {
    if (rows == 0) {
        print "ERROR: no table rows parsed" > "/dev/stderr"
        exit 1
    }
    # Round to 2 decimal places
    printf "%.2f\n", total
}
' "$PROGRESS")

if [ -z "$COMPUTED" ]; then
    echo "ERROR: failed to parse table in $PROGRESS" >&2
    exit 1
fi

# Read the current header score
HEADER_SCORE=$(grep -m1 '^Current score:' "$PROGRESS" | sed 's/Current score:[[:space:]]*//' | sed 's/%//')

if [ "${1:-}" = "--check" ]; then
    if [ "$HEADER_SCORE" = "$COMPUTED" ]; then
        echo "OK: Current score header matches computed total (${COMPUTED}%)"
        exit 0
    else
        echo "MISMATCH: header says ${HEADER_SCORE}%, table computes ${COMPUTED}%" >&2
        exit 1
    fi
fi

# Print result
echo "Solidworks-tier total: ${COMPUTED}% (was 65.0% on 2026-05-06)"

# Update the "Current score:" line in PROGRESS.md
if [ "$HEADER_SCORE" != "$COMPUTED" ]; then
    # Use a temp file for portability (no sed -i differences across platforms)
    TMPFILE=$(mktemp)
    sed "s/^Current score:.*$/Current score: ${COMPUTED}%/" "$PROGRESS" > "$TMPFILE"
    mv "$TMPFILE" "$PROGRESS"
    echo "Updated Current score in $PROGRESS"
fi
