#!/usr/bin/env bash

set -euo pipefail

year="$(date +%Y)"
yy="$(date +%y)"
doy="$(date +%j)"

base_name="hour${doy}0.${yy}n"
gz_file="${base_name}.gz"
url="https://cddis.nasa.gov/archive/gnss/data/hourly/${year}/${doy}/${gz_file}"

echo "Downloading URL: ${url}"

wget --auth-no-challenge "$url"

gzip -df "$gz_file"

echo "Saved ephemeris file: ${base_name}"
