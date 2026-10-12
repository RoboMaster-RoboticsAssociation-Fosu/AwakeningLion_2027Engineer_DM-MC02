[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectRoot
)

$ErrorActionPreference = 'Stop'

# CubeMX generates an MDK-ARM V5 FreeRTOS port even when the retained Keil
# project uses ARM Compiler 6. Keep the generated project on the single,
# shared ARMClang-compatible port and shared libraries after every code generation.
# Board-local owners and generated configuration stay in the calling project.
$projectRoot = [System.IO.Path]::GetFullPath($ProjectRoot)
$keilProjectPath = Join-Path $projectRoot 'MDK-ARM\CtrlBoard-H7_IMU.uvprojx'
$generatedPortPath = '../../Shared/Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c'
$generatedIncludePath = '../../Shared/Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F'
$armClangPortPath = '../../Shared/ThirdParty/FreeRTOS/ARM_CM4F_AC6/port.c'
$armClangIncludePath = '../../Shared/ThirdParty/FreeRTOS/ARM_CM4F_AC6'

function ConvertTo-NormalizedProjectPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    return ($Path.Replace('\', '/').Trim()).ToLowerInvariant()
}

# Convert only library references from an MDK-ARM project; application sources,
# callback dispatch and per-board configuration are never redirected.
function ConvertTo-SharedProjectPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    $normalized = $Path.Replace('\', '/')
    if ($normalized -match '^\.\./User/Algorithm/(Arm|VMC)(/|$)') {
        return $Path
    }
    if ($normalized -match '^\.\./User/[Bb]sp/(bsp_dwt|bsp_PWM)\.(c|h)$') {
        return '../../Shared/Bsp/' + [System.IO.Path]::GetFileName($normalized)
    }
    if ($normalized -eq '../User/APP/ins_types.h') {
        return '../../Shared/Protocol/BoardLink/ins_types.h'
    }
    $prefixes = @{
        '../Drivers/' = '../../Shared/Drivers/'
        '../Middlewares/' = '../../Shared/Middlewares/'
        '../User/Devices/' = '../../Shared/Devices/'
        '../User/Protocol/' = '../../Shared/Protocol/'
        '../User/Algorithm/' = '../../Shared/Algorithm/'
        '../User/ThirdParty/' = '../../Shared/ThirdParty/'
        '../User/Controller/' = '../../Shared/Controller/'
        '../User/Lib/' = '../../Shared/Lib/'
    }
    foreach ($prefix in $prefixes.Keys) {
        if ($normalized.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
            return $prefixes[$prefix] + $normalized.Substring($prefix.Length)
        }
    }
    return $Path
}

if (-not (Test-Path -LiteralPath $keilProjectPath -PathType Leaf)) {
    throw "Keil project not found: $keilProjectPath"
}

$originalProjectText = [System.IO.File]::ReadAllText($keilProjectPath)
$encodingDeclarationMatch = [regex]::Match($originalProjectText, 'encoding="[^"]+"')

$xml = [System.Xml.XmlDocument]::new()
$xml.PreserveWhitespace = $true
$xml.Load($keilProjectPath)

$target = $xml.SelectSingleNode('/Project/Targets/Target')
if ($null -eq $target) {
    throw 'Invalid Keil project: target node is missing.'
}

if (($target.uAC6 -ne '1') -or ($target.pCCUsed -notmatch '::V6\.')) {
    throw 'Keil project is not configured for ARM Compiler 6; refusing to patch it.'
}

$generatedPortKey = ConvertTo-NormalizedProjectPath $generatedPortPath
$armClangPortKey = ConvertTo-NormalizedProjectPath $armClangPortPath
$removedPortCount = 0
$armClangPortNodes = @()

foreach ($fileNode in @($xml.SelectNodes('/Project/Targets/Target/Groups/Group/Files/File'))) {
    $filePathNode = $fileNode.SelectSingleNode('FilePath')
    if ($null -eq $filePathNode) {
        continue
    }

    $filePathNode.InnerText = ConvertTo-SharedProjectPath $filePathNode.InnerText
    $filePathKey = ConvertTo-NormalizedProjectPath $filePathNode.InnerText
    if ($filePathKey -eq $generatedPortKey) {
        $precedingWhitespace = $fileNode.PreviousSibling
        [void]$fileNode.ParentNode.RemoveChild($fileNode)
        if (($null -ne $precedingWhitespace) -and
            ($precedingWhitespace.NodeType -eq [System.Xml.XmlNodeType]::Whitespace)) {
            [void]$precedingWhitespace.ParentNode.RemoveChild($precedingWhitespace)
        }
        $removedPortCount++
    }
    elseif ($filePathKey -eq $armClangPortKey) {
        $armClangPortNodes += $fileNode
    }
}

if ($armClangPortNodes.Count -eq 0) {
    throw "ARMClang FreeRTOS port is missing from the Keil project: $armClangPortPath"
}
if ($armClangPortNodes.Count -gt 1) {
    throw "ARMClang FreeRTOS port is registered more than once: $armClangPortPath"
}

# Assembly and per-file settings can also contain CubeMX library paths.
# Redirect every existing include list before applying the C compiler port rule.
foreach ($includeNode in @($xml.SelectNodes('//IncludePath'))) {
    if ([string]::IsNullOrWhiteSpace($includeNode.InnerText)) {
        continue
    }
    $mappedPaths = @($includeNode.InnerText -split ';' | ForEach-Object {
        ConvertTo-SharedProjectPath $_.Trim()
    })
    $includeNode.InnerText = $mappedPaths -join ';'
}

$includePathNode = $xml.SelectSingleNode('/Project/Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls/IncludePath')
if ($null -eq $includePathNode) {
    throw 'Invalid Keil project: target include-path node is missing.'
}

$generatedIncludeKey = ConvertTo-NormalizedProjectPath $generatedIncludePath
$armClangIncludeKey = ConvertTo-NormalizedProjectPath $armClangIncludePath
$includePaths = [System.Collections.Generic.List[string]]::new()
$seenIncludePaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)

foreach ($includePath in ($includePathNode.InnerText -split ';')) {
    $trimmedPath = $includePath.Trim()
    if ([string]::IsNullOrWhiteSpace($trimmedPath)) {
        continue
    }

    $trimmedPath = ConvertTo-SharedProjectPath $trimmedPath
    $includePathKey = ConvertTo-NormalizedProjectPath $trimmedPath
    if ($includePathKey -eq $generatedIncludeKey) {
        continue
    }
    if ($seenIncludePaths.Add($includePathKey)) {
        $includePaths.Add($trimmedPath)
    }
}

if (-not $seenIncludePaths.Contains($armClangIncludeKey)) {
    $includePaths.Add($armClangIncludePath)
}
foreach ($sharedInclude in @('../../Shared/Bsp', '../../Shared/Protocol/BoardLink')) {
    if ($seenIncludePaths.Add((ConvertTo-NormalizedProjectPath $sharedInclude))) {
        $includePaths.Add($sharedInclude)
    }
}
$includePathNode.InnerText = $includePaths -join ';'

$writerSettings = [System.Xml.XmlWriterSettings]::new()
$writerSettings.Encoding = [System.Text.UTF8Encoding]::new($false)
$writerSettings.Indent = $false
$writerSettings.NewLineHandling = [System.Xml.NewLineHandling]::None
$writer = [System.Xml.XmlWriter]::Create($keilProjectPath, $writerSettings)
try {
    $xml.Save($writer)
}
finally {
    $writer.Dispose()
}

# XmlWriter canonicalizes the encoding name and may choose a different newline
# for the declaration. Restore the generator's text conventions to keep diffs
# limited to the actual project-node changes.
$patchedProjectText = [System.IO.File]::ReadAllText($keilProjectPath)
if ($encodingDeclarationMatch.Success) {
    $patchedProjectText = [regex]::Replace(
        $patchedProjectText,
        'encoding="[^"]+"',
        $encodingDeclarationMatch.Value,
        1)
}
[System.IO.File]::WriteAllText(
    $keilProjectPath,
    $patchedProjectText,
    [System.Text.UTF8Encoding]::new($false))

Write-Host "[CubeMX post-generate] Kept Shared source references and ARMClang FreeRTOS port; removed $removedPortCount generated RVDS port entry."
