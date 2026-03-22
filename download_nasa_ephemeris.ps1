[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Cookie = $env:NASA_AUTH_COOKIE
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($Cookie)) {
    Write-Host "Usage: `$env:NASA_AUTH_COOKIE='urs_guid_ops=...; ProxyAuth=...'; .\download_nasa_ephemeris.ps1"
    Write-Host "   or: .\download_nasa_ephemeris.ps1 'urs_guid_ops=...; ProxyAuth=...'"
    exit 1
}

$today = Get-Date
$year = $today.ToString("yyyy")
$yy = $today.ToString("yy")
$dayOfYear = $today.DayOfYear.ToString("000")

$baseName = "brdc${dayOfYear}0.${yy}n"
$gzFile = "${baseName}.gz"
$url = "https://cddis.nasa.gov/archive/gnss/data/daily/${year}/${dayOfYear}/${yy}n/${gzFile}"

Write-Host "Downloading URL: $url"

& curl.exe --fail --location $url --cookie $Cookie --output $gzFile

if ($LASTEXITCODE -ne 0) {
    throw "curl download failed with exit code $LASTEXITCODE"
}

& gzip.exe -df $gzFile

if ($LASTEXITCODE -ne 0) {
    throw "gzip extraction failed with exit code $LASTEXITCODE"
}

Write-Host "Saved ephemeris file: $baseName"
