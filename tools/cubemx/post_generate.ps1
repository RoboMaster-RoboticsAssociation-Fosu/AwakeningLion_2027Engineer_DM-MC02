[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

# CubeMX generates an MDK-ARM V5 FreeRTOS port even when the retained Keil
# project uses ARM Compiler 6. Keep the generated project on the single,
# repository-owned ARMClang-compatible port after every code generation.
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$keilProjectPath = Join-Path $projectRoot 'MDK-ARM\CtrlBoard-H7_IMU.uvprojx'
$generatedPortPath = '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c'
$generatedIncludePath = '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F'
$armClangPortPath = '../User/ThirdParty/FreeRTOS/ARM_CM4F_AC6/port.c'
$armClangIncludePath = '../User/ThirdParty/FreeRTOS/ARM_CM4F_AC6'

function ConvertTo-NormalizedProjectPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    return ($Path.Replace('\', '/').Trim()).ToLowerInvariant()
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

Write-Host "[CubeMX post-generate] Kept ARMClang FreeRTOS port and removed $removedPortCount generated RVDS port entry."
