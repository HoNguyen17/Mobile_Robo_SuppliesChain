using System;
using System.Diagnostics;
using System.IO;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Menu items for one-click A/B benchmarking between A* and D* Lite planners.
/// Executes ab_batch.py in WSL asynchronously and streams logs to Unity console.
/// Conservative C# for Unity 2021.1 (.NET Standard 2.0).
/// </summary>
public static class AbBenchmarkMenu
{
    // Distribution name must match the output of `wsl -l -v`
    private const string WslDistro = "Ubuntu-20.04";

    private static volatile bool isBatchRunning = false;
    private static Process activeProcess = null;

    public static bool IsBatchRunning
    {
        get { return isBatchRunning; }
    }

    // --- Validation Methods (Items 1-4 enabled only in Play mode and when not running) ---

    [MenuItem("Warehouse/A-B Test/1 Dry Run (A* then D*, 3 each)", true)]
    public static bool ValidateDryRun()
    {
        return EditorApplication.isPlaying && !isBatchRunning;
    }

    [MenuItem("Warehouse/A-B Test/2 Run A* Batch (5)", true)]
    public static bool ValidateAstarBatch()
    {
        return EditorApplication.isPlaying && !isBatchRunning;
    }

    [MenuItem("Warehouse/A-B Test/3 Run D* Lite Batch (5)", true)]
    public static bool ValidateDstarBatch()
    {
        return EditorApplication.isPlaying && !isBatchRunning;
    }

    [MenuItem("Warehouse/A-B Test/4 Run Both + Report (5 each)", true)]
    public static bool ValidateBothBatch()
    {
        return EditorApplication.isPlaying && !isBatchRunning;
    }

    // --- Menu Commands ---

    [MenuItem("Warehouse/A-B Test/1 Dry Run (A* then D*, 3 each)")]
    public static void RunDryRun()
    {
        StartBatchAsync("--planner both --dry", openReportOnFinish: false);
    }

    [MenuItem("Warehouse/A-B Test/2 Run A* Batch (5)")]
    public static void RunAstarBatch()
    {
        StartBatchAsync("--planner astar --repeats 5", openReportOnFinish: true);
    }

    [MenuItem("Warehouse/A-B Test/3 Run D* Lite Batch (5)")]
    public static void RunDstarBatch()
    {
        StartBatchAsync("--planner dstar --repeats 5", openReportOnFinish: true);
    }

    [MenuItem("Warehouse/A-B Test/4 Run Both + Report (5 each)")]
    public static void RunBothBatch()
    {
        StartBatchAsync("--planner both --repeats 5", openReportOnFinish: true);
    }

    [MenuItem("Warehouse/A-B Test/5 Open Latest Report")]
    public static void OpenLatestReportMenu()
    {
        OpenLatestReport();
    }

    [MenuItem("Warehouse/A-B Test/6 Stop Batch")]
    public static void StopBatch()
    {
        UnityEngine.Debug.Log("[A/B] Requesting batch stop via ab_batch.py --stop...");
        RunWslFireAndForget("--stop");
    }

    [MenuItem("Warehouse/A-B Test/7 Open Latest Session Folder")]
    public static void OpenLatestSessionFolder()
    {
        string readCmd = "cat ~/ab_runs/latest_session.txt 2>/dev/null";
        string linuxSessionPath = RunWslCommandSync(readCmd).Trim();

        if (string.IsNullOrEmpty(linuxSessionPath))
        {
            UnityEngine.Debug.LogWarning("[A/B] No session found in ~/ab_runs/latest_session.txt");
            return;
        }

        // Convert WSL path to Windows path using wslpath -w
        string winPath = RunWslCommandSync("wslpath -w " + linuxSessionPath).Trim();

        if (!string.IsNullOrEmpty(winPath) && Directory.Exists(winPath))
        {
            UnityEngine.Debug.Log("[A/B] Opening session folder: Linux=" + linuxSessionPath + " | Windows=" + winPath);
            try
            {
                Process.Start("explorer.exe", "\"" + winPath + "\"");
            }
            catch (Exception ex)
            {
                UnityEngine.Debug.LogWarning("[A/B] Process.Start failed to open explorer: " + ex.Message + ". Path: " + winPath);
            }
        }
        else
        {
            UnityEngine.Debug.LogWarning("[A/B] Session directory could not be resolved on Windows. Linux path: " + linuxSessionPath + (string.IsNullOrEmpty(winPath) ? "" : " | Windows: " + winPath));
        }
    }

    // --- Process Execution Helpers ---

    private static string GetRepoWindowsPath()
    {
        // Application.dataPath points to <repo>/WarehouseProjectURP/Assets
        return Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));
    }

    public static string BuildWslCommandLine(string extraArgs)
    {
        string repoWinPath = GetRepoWindowsPath();
        string bashCmd = "source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && python3 -u ros1/turtlebot_control/scripts/ab_batch.py " + extraArgs;
        return "-d " + WslDistro + " --cd \"" + repoWinPath + "\" -e bash -c \"" + bashCmd + "\"";
    }

    private static void StartBatchAsync(string extraArgs, bool openReportOnFinish)
    {
        if (isBatchRunning)
        {
            UnityEngine.Debug.LogWarning("[A/B] A batch run is already in progress. Stop it first or wait for completion.");
            return;
        }

        string arguments = BuildWslCommandLine(extraArgs);
        UnityEngine.Debug.Log("[A/B] Starting batch: wsl.exe " + arguments);

        RunWslCommandAsync(arguments, openReportOnFinish);
    }

    private static void RunWslFireAndForget(string extraArgs)
    {
        try
        {
            ProcessStartInfo psi = new ProcessStartInfo
            {
                FileName = "wsl.exe",
                Arguments = BuildWslCommandLine(extraArgs),
                UseShellExecute = false,
                CreateNoWindow = true
            };
            Process.Start(psi);
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError("[A/B] Failed to launch WSL stop process: " + ex.Message);
        }
    }

    private static void RunWslCommandAsync(string arguments, bool openReportOnFinish)
    {
        try
        {
            ProcessStartInfo psi = new ProcessStartInfo
            {
                FileName = "wsl.exe",
                Arguments = arguments,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true
            };

            Process proc = new Process();
            proc.StartInfo = psi;
            proc.EnableRaisingEvents = true;

            proc.OutputDataReceived += (sender, args) =>
            {
                if (!string.IsNullOrEmpty(args.Data))
                {
                    string line = args.Data;
                    UnityEngine.Debug.Log("[A/B] " + line);

                    if (line.Contains("Session directory:"))
                    {
                        int idx = line.IndexOf("Session directory:") + "Session directory:".Length;
                        string sessionDir = line.Substring(idx).Trim();
                        string batchLog = sessionDir + "/batch.log";
                        UnityEngine.Debug.Log("[A/B] Batch log: " + batchLog);
                    }
                }
            };

            proc.ErrorDataReceived += (sender, args) =>
            {
                if (!string.IsNullOrEmpty(args.Data))
                {
                    UnityEngine.Debug.LogWarning("[A/B] " + args.Data);
                }
            };

            proc.Exited += (sender, args) =>
            {
                isBatchRunning = false;
                activeProcess = null;
                int exitCode = proc.ExitCode;
                UnityEngine.Debug.Log("[A/B] Process exited with code: " + exitCode);
                proc.Dispose();

                if (openReportOnFinish && exitCode == 0)
                {
                    EditorApplication.delayCall += () =>
                    {
                        OpenLatestReport();
                    };
                }
            };

            isBatchRunning = true;
            activeProcess = proc;
            proc.Start();
            proc.BeginOutputReadLine();
            proc.BeginErrorReadLine();
        }
        catch (Exception ex)
        {
            isBatchRunning = false;
            activeProcess = null;
            UnityEngine.Debug.LogError("[A/B] Failed to launch WSL process: " + ex.Message);
        }
    }

    private static string RunWslCommandSync(string bashCmd)
    {
        try
        {
            ProcessStartInfo psi = new ProcessStartInfo
            {
                FileName = "wsl.exe",
                Arguments = "-d " + WslDistro + " -e bash -c \"" + bashCmd.Replace("\"", "\\\"") + "\"",
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true
            };

            using (Process proc = Process.Start(psi))
            {
                if (proc == null) return string.Empty;
                string output = proc.StandardOutput.ReadToEnd();
                proc.WaitForExit(5000);
                return output;
            }
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogWarning("[A/B] Error executing synchronous WSL command: " + ex.Message);
            return string.Empty;
        }
    }

    public static void OpenLatestReport()
    {
        string readCmd = "cat ~/ab_runs/latest_report.txt 2>/dev/null";
        string linuxReportPath = RunWslCommandSync(readCmd).Trim();

        if (string.IsNullOrEmpty(linuxReportPath))
        {
            UnityEngine.Debug.LogWarning("[A/B] No report found in ~/ab_runs/latest_report.txt");
            return;
        }

        // Convert WSL path to Windows path using wslpath -w
        string winPath = RunWslCommandSync("wslpath -w \"" + linuxReportPath + "\" 2>/dev/null").Trim();

        if (!string.IsNullOrEmpty(winPath) && File.Exists(winPath))
        {
            UnityEngine.Debug.Log("[A/B] Opening report in Windows browser: " + winPath);
            try
            {
                Process.Start(new ProcessStartInfo
                {
                    FileName = winPath,
                    UseShellExecute = true
                });
            }
            catch (Exception ex)
            {
                UnityEngine.Debug.LogWarning("[A/B] Process.Start failed to open browser: " + ex.Message + ". Path: " + winPath);
            }
        }
        else
        {
            UnityEngine.Debug.LogWarning("[A/B] Report path could not be resolved on Windows. Linux path: " + linuxReportPath + (string.IsNullOrEmpty(winPath) ? "" : " | Windows: " + winPath));
        }
    }
}
