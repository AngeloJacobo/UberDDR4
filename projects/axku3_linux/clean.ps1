# Remove this example's local generated files; no Python/Vivado installation needed.
# Default: build/output. -All: build, including downloaded dependencies/tools.
# Deliberately ignore local.ps1 and external cache overrides: never recursively
# delete a user-configured external path. Stop builds and save wanted logs first.
[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'High')]
param([switch]$All)

$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath($PSScriptRoot)
$cacheRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'build'))
$target = if ($All) { $cacheRoot } else { Join-Path $cacheRoot 'output' }
$target = [IO.Path]::GetFullPath($target)
if (-not $target.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar,
        [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Cleanup target is outside this project.'
}

# Reject junctions/symlinks in the path before accessing the target's contents.
# A linked build directory must never turn cleanup into deletion elsewhere.
$ancestor = $target
while ($ancestor) {
    if (Test-Path -LiteralPath $ancestor) {
        $item = Get-Item -LiteralPath $ancestor -Force
        if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {
            throw "Refusing linked cleanup path: $ancestor"
        }
    }
    $ancestor = [IO.Path]::GetDirectoryName($ancestor)
}
if (-not (Test-Path -LiteralPath $target)) {
    Write-Host "Nothing to clean: $target"
    return
}
if (-not (Test-Path -LiteralPath $target -PathType Container)) {
    throw "Expected a build directory, not a file: $target"
}

# Walk explicitly rather than following a recursive filesystem traversal.
# Refuse any nested link before deleting anything, including under -WhatIf.
$pending = [Collections.Generic.Stack[string]]::new()
$pending.Push($target)
while ($pending.Count) {
    foreach ($item in Get-ChildItem -LiteralPath $pending.Pop() -Force) {
        if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {
            throw "Refusing linked cleanup entry: $($item.FullName)"
        }
        if ($item.PSIsContainer) { $pending.Push($item.FullName) }
    }
}

$action = if ($All) {
    'Permanently delete ALL local dependencies, tools, build outputs and test logs'
} else {
    'Permanently delete generated outputs, including bitstreams and test logs'
}
if ($PSCmdlet.ShouldProcess($target, $action)) {
    Remove-Item -LiteralPath $target -Recurse -Force -Confirm:$false
    Write-Host "Removed permanently: $target"
}
