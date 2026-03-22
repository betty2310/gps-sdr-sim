#!/usr/bin/env bash

set -euo pipefail

cookie="$1"

year="$(date +%Y)"
yy="$(date +%y)"
doy="$(date +%j)"

base_name="brdc${doy}0.${yy}n"
gz_file="${base_name}.gz"
url="https://cddis.nasa.gov/archive/gnss/data/daily/${year}/${doy}/${yy}n/${gz_file}"

echo "Downloading URL: ${url}"

curl "$url" \
  -b "ProxyAuth=${cookie}" \
  --output "$gz_file"

gzip -df "$gz_file"

echo "Saved ephemeris file: ${base_name}"
