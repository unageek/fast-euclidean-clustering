Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

. .\tools\Exec.ps1
. .\tools\Invoke-BatchFile.ps1

function getExternalArguments {
    param ([String[]]$arguments)
    [String[]]$extArgs = @()
    $i = [array]::indexof($arguments, "--")
    if ( $i -ne -1 ) {
        $__, $extArgs = $arguments[$i..($arguments.length - 1)]
    }
    $extArgs
}

function loadBuildEnvironment {
    $vswhere = Resolve-Path "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"

    $vsDir = & $vswhere -latest `
        -requires Microsoft.VisualStudio.Workload.NativeDesktop `
        -property installationPath

    if (-not $vsDir) {
        throw 'Some of the required workloads/components of Visual Studio are not installed.'
    }

    Invoke-BatchFile "$vsDir\VC\Auxiliary\Build\vcvars64.bat"
}

if ($args.Length -lt 1) {
    throw 'No task is specified.'
}

Set-Location $PSScriptRoot
$errorMsg = "Command execution failed"

switch -regex ($args[0]) {
    '^init-vcpkg$' {
        Set-Location vcpkg
        Exec { .\bootstrap-vcpkg.bat } $errorMsg
        Exec { .\vcpkg remove --outdated --recurse } $errorMsg
        Exec { .\vcpkg install '@..\vcpkg.txt' --triplet=x64-windows } $errorMsg
        break
    }
    '^cmake$' {
        loadBuildEnvironment
        New-Item build -ItemType Directory -Force
        Set-Location build
        $extArgs = getExternalArguments $args
        Exec { cmake .. -GNinja $extArgs } $errorMsg
        break
    }
    '^b(uild)?$' {
        loadBuildEnvironment
        Set-Location build
        Exec { ninja } $errorMsg
        break
    }
    '^r(un)?$' {
        $bin = $args[1]
        $extArgs = getExternalArguments $args
        Exec { & .\build\$bin $extArgs } $errorMsg
        break
    }
    '^t(est)?$' {
        loadBuildEnvironment
        Set-Location build
        Exec { ctest -V } $errorMsg
        break
    }
    default {
        throw "Unknown task: '$($args[0])'"
    }
}
