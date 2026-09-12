# ADR-007: Development Environment — Ubuntu for the Whole Stack, on Every Machine

## Context
The document set pins a ROS 2 distribution in Phase 0 (ADR-003) but never states the host operating system that distribution runs on, leaving a gap that every other Phase 0 task silently depends on: the tutorial repository cannot be cloned and verified, and `docker-compose up` cannot be dry-run, without an environment to do it in.

ROS 2 and Nav2 are supported in practice on Ubuntu; native Windows ROS 2 support exists on paper but is not a maintained path for a Nav2-based stack and would put the graded milestones on unsupported ground. Unity, meanwhile, ships an Editor for Linux as well as Windows, and the Unity↔ROS bridge is a TCP connection (`docs/architecture.md` §2), so Unity and ROS 2 are not technically required to share a machine or an OS.

That technical freedom is narrower than it looks. A dual-boot Ubuntu runs one OS at a time, so a Unity-on-Windows / ROS-on-Ubuntu split is only possible with a virtual machine, WSL2, or a second physical machine — it is not something dual-booting can deliver. The team is four students on their own hardware, and the Phase 0 exit gate requires `docker-compose up` to succeed on every member's machine, so environment variance is a direct threat to that gate.

## Decision
The entire stack — Unity Editor, ROS 2, Nav2, Docker/MySQL, and the headless runner — runs on **Ubuntu**, on every team member's machine, using the Ubuntu LTS release paired with the ROS 2 distribution pinned by ADR-003. Dual-boot is the expected installation method.

Unity runs under the Linux Editor rather than on Windows. Source editing may happen from any OS (e.g. VS Code Remote-SSH), but nothing that ROS 2 touches executes outside Ubuntu.

**WSL2 fallback:** a team member whose hardware genuinely cannot dual-boot may run ROS 2 and Docker under WSL2 with Unity on Windows, connecting over the TCP bridge. This is a per-person accommodation, not a second team standard: that member's results do not count toward a gate until reproduced on a native Ubuntu machine, because virtualised timing is not comparable for the latency- and throughput-sensitive metrics in `docs/test-plan.md`.

[VERIFY] — Phase 0 task: confirm the Ubuntu LTS release paired with the ADR-003 distro, confirm Unity Editor for Linux runs the Unity Robotics Hub tutorial scene, and confirm every machine has sufficient disk headroom for the partition. Record the result here as a status update to this ADR, not as a new ADR.

## Consequences
- ADR-003 is unblocked: the distro cannot be verified against the tutorial repository until an Ubuntu environment exists, so this decision is upstream of every other Phase 0 task and must be completed first.
- The Ubuntu LTS release is not independently chosen — it follows from the ADR-003 distro pin, so the two decisions are finalised together.
- One environment across four people means one set of environment bugs. Mixed-OS debugging, which would otherwise consume time from a graded path in an 8-week timeline, is designed out.
- The headless runner (`docs/test-plan.md` §3) executes natively. Given N≥20 runs across six scenarios, plus the DWB-vs-TEB benchmark (ADR-004) and the 60-run S-06 matrix, the absence of a virtualisation tax is a schedule factor, not a preference.
- Multi-robot Stage B/C (ADR-006) runs 2–3 full Nav2 stacks concurrently alongside Unity; native execution preserves the headroom that staging depends on.
- Docker runs natively rather than through a Linux VM, simplifying the `docker-compose` + `schema.sql` path (ADR-005).
- The team accepts the Unity Linux Editor's rougher edges relative to the Windows Editor, which is a smaller cost than an unsupported ROS 2 path would be.
- Each member must allocate a disk partition sized for Ubuntu, the Unity Editor, and ROS 2 desktop-full. Insufficient headroom is a Phase 0 blocker, surfaced before M1 work begins, not after.

## Alternatives Rejected
- **Unity on Windows, ROS 2 on dual-boot Ubuntu:** rejected — not physically possible, since dual-boot runs one OS at a time. The split requires a VM, WSL2, or second machine.
- **Native ROS 2 on Windows:** rejected — Nav2 on Windows is not a maintained path, and placing the graded navigation stack on unsupported ground contradicts the whole point of ADR-003's refusal to build on unverified assumptions.
- **WSL2 as the team standard:** rejected as the default — it works, but adds a virtualisation and networking layer beneath latency-sensitive measurements, on a project whose grade is numeric evidence. Retained as a per-person fallback under the constraint stated above.
- **Let each member choose their own OS:** rejected — directly undermines the Phase 0 exit gate's "on every team member's machine" requirement, and multiplies environment-specific failures across a 4-person team with no slack in the timeline.
- **Defer the OS decision to whenever each member starts work:** rejected — it is the precondition for every other Phase 0 task, including the ADR-003 verification that all of Phase 1 depends on.
