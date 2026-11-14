// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

typedef void b2TaskCallback( void* taskContext );
typedef struct b2Scheduler b2Scheduler;

b2Scheduler* b2CreateScheduler( int workerCount );
void b2DestroyScheduler( b2Scheduler* scheduler );
void b2ResetScheduler( b2Scheduler* scheduler );

// WeedJS E-298: 1 while a world step runs (its workers spin between phases), 0 between steps (they may sleep).
void b2SchedulerSetStepping( b2Scheduler* scheduler, int stepping );

// See b2EnqueueTaskCallback and b2FinishTaskCallback
void* b2SchedulerEnqueueTask( b2TaskCallback* task, void* taskContext, void* userContext );
void b2SchedulerFinishTask( void* userTask, void* userContext );
