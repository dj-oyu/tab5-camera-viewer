# Update Documentation

Sync documentation with the codebase after architectural or configuration changes.

## Step 1: Identify What Changed

Check recent git commits and changed files:

```bash
git diff --name-only HEAD~5..HEAD
```

Identify if changes affect:
- Pipeline architecture (buffer sizes, task priorities, queue depths)
- Memory layout (SPIRAM allocations)
- Task interactions (new queues, semaphores, events)
- Build configuration (`platformio.ini`, `PipelineConfig.h`)
- Environment variables (`.env` template)

## Step 2: Update Pipeline Architecture Doc

If pipeline structure, buffer sizes, or memory layout changed:

1. Read `docs/pipeline-architecture.md`
2. Update the affected sections:
   - **Memory Layout** (if buffer sizes changed in `PipelineConfig.h`)
   - **3-Stage Pipeline** diagram (if tasks/queues added/removed)
   - **Display Layout** (if layout changed)

## Step 3: Update Task Interactions Doc

If task communication changed (new queues, semaphores, SSE streams):

1. Read `docs/task-interactions.md`
2. Update:
   - Queue/semaphore tables
   - Task dependency flows
   - Inter-task communication patterns

## Step 4: Update CLAUDE.md

If key files, architecture summary, or memory layout changed:

1. Read `CLAUDE.md`
2. Update the **Key Files** table if new files are significant
3. Update **Memory Layout** section if buffer sizes changed
4. Update **Architecture** section if pipeline structure changed

## Step 5: Show Summary

```
Documentation Update
──────────────────────────────
Updated:  docs/pipeline-architecture.md (memory layout)
Updated:  CLAUDE.md (key files table)
Skipped:  docs/task-interactions.md (no task changes)
──────────────────────────────
```

## Rules

- **Single source of truth**: Code is the truth; docs describe the code
- **Preserve existing prose**: Only update sections that are factually outdated
- **Don't create new docs unprompted**: Only update existing files unless explicitly requested
- **Be specific**: Include actual numbers (buffer sizes in KB, queue depths, priorities)
