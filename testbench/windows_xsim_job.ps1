param(
    [Parameter(Mandatory = $true)]
    [string]$RepositoryRoot,

    [Parameter(Mandatory = $true)]
    [int]$TimeoutSeconds,

    [Parameter(Mandatory = $true)]
    [string]$StopFile
)

$ErrorActionPreference = 'Stop'

# A Windows Job Object gives the simulator one reliable lifetime boundary.
# When this PowerShell process exits for any reason, KILL_ON_JOB_CLOSE removes
# Bash and every Vivado/XSim descendant without depending on MSYS parent PIDs.
if (-not ('UberDDR4.NativeJob' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;

namespace UberDDR4 {
    public static class NativeJob {
        [StructLayout(LayoutKind.Sequential)]
        public struct IO_COUNTERS {
            public UInt64 ReadOperationCount, WriteOperationCount, OtherOperationCount;
            public UInt64 ReadTransferCount, WriteTransferCount, OtherTransferCount;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct BASIC_LIMIT_INFORMATION {
            public Int64 PerProcessUserTimeLimit, PerJobUserTimeLimit;
            public UInt32 LimitFlags;
            public UIntPtr MinimumWorkingSetSize, MaximumWorkingSetSize;
            public UInt32 ActiveProcessLimit;
            public UIntPtr Affinity;
            public UInt32 PriorityClass, SchedulingClass;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct EXTENDED_LIMIT_INFORMATION {
            public BASIC_LIMIT_INFORMATION BasicLimitInformation;
            public IO_COUNTERS IoInfo;
            public UIntPtr ProcessMemoryLimit, JobMemoryLimit, PeakProcessMemoryUsed, PeakJobMemoryUsed;
        }

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode)]
        public static extern IntPtr CreateJobObject(IntPtr attributes, string name);

        [DllImport("kernel32.dll")]
        public static extern bool SetInformationJobObject(
            IntPtr job, int infoClass, IntPtr info, UInt32 length);

        [DllImport("kernel32.dll")]
        public static extern bool AssignProcessToJobObject(IntPtr job, IntPtr process);

        [DllImport("kernel32.dll")]
        public static extern bool TerminateJobObject(IntPtr job, UInt32 exitCode);

        [DllImport("kernel32.dll")]
        public static extern bool CloseHandle(IntPtr handle);

        public const UInt32 JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x00002000;
        public const int JobObjectExtendedLimitInformation = 9;
    }
}
'@
}

$job = [UberDDR4.NativeJob]::CreateJobObject([IntPtr]::Zero, $null)
if ($job -eq [IntPtr]::Zero) {
    throw 'CreateJobObject failed.'
}

$process = $null
$result = 1
$startedAt = Get-Date

function Stop-OwnedXilinxTools {
    # Vivado's loader can break xvlog/xelab/xsim out of an inherited Job.
    # The regression lock guarantees one suite per workspace; restrict this
    # fallback further to simulator tools created during this owned run.
    $names = @('xvlog', 'xelab', 'xsim', 'xsimk')
    Get-Process -ErrorAction SilentlyContinue | Where-Object {
        $names -contains $_.ProcessName -and
        $_.StartTime -ge $startedAt.AddSeconds(-2)
    } | Stop-Process -Force -ErrorAction SilentlyContinue
}

try {
    Remove-Item -LiteralPath $StopFile -Force -ErrorAction SilentlyContinue
    $limits = New-Object UberDDR4.NativeJob+EXTENDED_LIMIT_INFORMATION
    $limits.BasicLimitInformation.LimitFlags =
        [UberDDR4.NativeJob]::JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    $size = [Runtime.InteropServices.Marshal]::SizeOf($limits)
    $buffer = [Runtime.InteropServices.Marshal]::AllocHGlobal($size)
    try {
        [Runtime.InteropServices.Marshal]::StructureToPtr($limits, $buffer, $false)
        if (-not [UberDDR4.NativeJob]::SetInformationJobObject(
            $job,
            [UberDDR4.NativeJob]::JobObjectExtendedLimitInformation,
            $buffer,
            $size)) {
            throw 'SetInformationJobObject failed.'
        }
    }
    finally {
        [Runtime.InteropServices.Marshal]::FreeHGlobal($buffer)
    }

    $bash = (Get-Command bash.exe -ErrorAction Stop).Source
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = $bash
    $start.Arguments = 'testbench/run_xsim.sh'
    $start.WorkingDirectory = $RepositoryRoot
    $start.UseShellExecute = $false

    $process = New-Object Diagnostics.Process
    $process.StartInfo = $start
    if (-not $process.Start()) {
        throw 'Unable to start testbench/run_xsim.sh.'
    }

    if (-not [UberDDR4.NativeJob]::AssignProcessToJobObject($job, $process.Handle)) {
        $process.Kill()
        throw 'AssignProcessToJobObject failed.'
    }

    $deadline = $startedAt.AddSeconds($TimeoutSeconds)
    while (-not $process.HasExited) {
        if (Test-Path -LiteralPath $StopFile) {
            [void][UberDDR4.NativeJob]::TerminateJobObject($job, 130)
            Start-Sleep -Milliseconds 100
            Stop-OwnedXilinxTools
            $result = 130
            break
        }
        if ((Get-Date) -ge $deadline) {
            [Console]::Error.WriteLine(
                "TIMEOUT: simulation exceeded {0} minutes", [Math]::Ceiling($TimeoutSeconds / 60.0))
            [void][UberDDR4.NativeJob]::TerminateJobObject($job, 124)
            Start-Sleep -Milliseconds 100
            Stop-OwnedXilinxTools
            $result = 124
            break
        }
        Start-Sleep -Milliseconds 250
    }
    if ($process.HasExited -and $result -eq 1) {
        $result = $process.ExitCode
    }
}
finally {
    # Closing the last job handle is the only tree-cleanup operation needed.
    [void][UberDDR4.NativeJob]::CloseHandle($job)
    if ($null -ne $process) {
        $process.Dispose()
    }
    Remove-Item -LiteralPath $StopFile -Force -ErrorAction SilentlyContinue
}

exit $result
