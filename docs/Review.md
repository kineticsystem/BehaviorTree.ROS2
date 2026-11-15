# BehaviorTree.ROS2 Review

## Table of Contents <!-- omit in toc -->

- [Scope and Method](#scope-and-method)
- [Verdict](#verdict)
- [The Fork's Commit](#the-forks-commit)
  - [Every Path Through execute()](#every-path-through-execute)
  - [What It Leaves Open](#what-it-leaves-open)
- [Under a Server That Moves Motors](#under-a-server-that-moves-motors)
  - [A Halt at Any Moment](#a-halt-at-any-moment)
  - [A Slow or Absent Action Server](#a-slow-or-absent-action-server)
  - [A Second Goal](#a-second-goal)
  - [Shutdown During a Run](#shutdown-during-a-run)
- [Structure and Dependencies](#structure-and-dependencies)
- [Keeping the Fork](#keeping-the-fork)
- [Testability and Tests](#testability-and-tests)
- [Implementation Issues](#implementation-issues)
- [Suspicions the Tests Disproved](#suspicions-the-tests-disproved)
- [What the Documents Say and the Tests Do Not](#what-the-documents-say-and-the-tests-do-not)
- [Recommendations](#recommendations)

Reviewed on 2026-11-15, at commit `b0ae01d`, the head of the fork's `humble`. It judges what [docs/ARCHITECTURE.md](ARCHITECTURE.md) describes, and does not repeat it.

## Scope and Method

This is the review of a fork, not of upstream's design. The repository is upstream's BehaviorTree.ROS2 at `6c6aa07` plus one commit, `b0ae01d`, 43 lines added and 3 removed. The review answers four questions:

1. Is that commit correct and complete?
2. Is the library safe under a server whose trees move motors, on the paths such a server takes: a halt at any moment, a slow or absent action server, a second goal, a shutdown during a run?
3. Is keeping the fork right, and what should go upstream?
4. What is tested?

We read every file of `behaviortree_ros2`, 2558 lines: `tree_execution_server.cpp` and `bt_utils.cpp`, the four node wrappers, `plugins.hpp`, `ros_node_params.hpp` and the logger. `btcpp_ros2_interfaces` is 4 interface files. `btcpp_ros2_samples` was not reviewed: no user builds it. We also read the two users named in [Who Uses It](ARCHITECTURE.md#who-uses-it), where they meet the library: the server of StepIt Commander, which derives from `TreeExecutionServer` and replaces a running tree through `onGoalReceived()` and `onLoopAfterTick()`, and the behaviours of StepIt Macro, a focus stacking rig, which derive from `RosServiceNode` and from a `RosActionNode` of their own whose `halt()` drops `UnknownGoalHandleError`.

A finding marked **verified** was reproduced by a throwaway program, since deleted; the others are **from reading**. The program ran inside the image of StepIt Macro, built in a scratch folder on top of the installed library, so that nothing was added to the repository:

- **The versions measured**: ROS 2 Jazzy, `rclcpp` 28.1.22, BehaviorTree.CPP 4.10.0, `generate_parameter_library` 0.7.6.
- **The server**: a subclass of `TreeExecutionServer` whose hooks throw on demand, spun by a `MultiThreadedExecutor` as StepIt Commander's `main()` does, and an `ExecuteTree` client.
- **The nodes**: nodes that throw at a chosen tick or when halted, a node that runs for ever and counts its halts, and a `RosActionNode` of `btcpp_ros2_interfaces/action/Sleep`, plain and with a copy of StepIt Macro's `halt()`.
- **The action server**: a fake `Sleep` server in a ROS context of its own, which can answer a goal late, refuse a cancel, hang or disappear, and counts the goals it receives, cancels and completes.
- **One scenario per process**, 38 of them, since 11 end the process. The use-after-free was confirmed with a second build under `-fsanitize=address`.

Not done: ThreadSanitizer, because the races of [Implementation Issues](#implementation-issues) sit inside BehaviorTree.CPP's shared library, which is not instrumented; and the test of StepIt Commander that covers the fork's change, `ATreeThatThrowsEndsItsRun`, was read, not run. The repository has no test of its own, so there was no suite to run: 0 tests.

## Verdict

**The fork's commit is correct for what it claims, and it is not the end of the job.** On the 9 paths that leave the process alive, the tree is halted and `onTreeExecutionCompleted()` is called exactly once, never twice. Four paths still end the process, and a halt that throws leaves the nodes after it running.

**The larger risk is not in `execute()`; it is in `RosActionNode`, which the fork does not touch.** Three verified defects of upstream:

- A node that runs a second time and is halted before its new goal is answered tries to cancel its previous goal. The new goal is never cancelled and runs to its end on its server, while the tree reports that it was halted.
- The callbacks of that goal still point to the tree node. When the next goal replaces the tree, they run on a destroyed object: the server crashes.
- A halt waits on the tick thread for the action server, 2.0 s to 2.8 s at a `server_timeout` of 1 s when the server hangs, and then leaves without having cancelled anything.

**For StepIt Macro none of them moves a motor**: its motion does not go through an action, and the one action its trees use fires a camera and its lights. The cost there is a shot fired after a stop, and a server that can crash at the next run. A server that dies cannot serve the next stop either, which is why we count the crash as a safety issue. A user whose trees move a robot through `RosActionNode` has a direct safety issue: a motion that goes on after a halt.

**Keeping the fork is right, and its policy needs to change.** Upstream's last commit is of 2025-11-25, and the fork's pull request there, 135, was closed unmerged ten minutes after it was opened. The fixes above belong in `bt_action_node.hpp`, where the members they need are private: no wrapper in a user can make them.

## The Fork's Commit

### Every Path Through execute()

Each row was run. "Halted" is whether a node still running in the tree received its halt; "hook" is the calls of `onTreeExecutionCompleted()`.

| What throws, and when | Halted | Hook | The goal | The process |
|---|---|---|---|---|
| a node, at its first tick | yes | once, `(FAILURE, false)` | aborted, `FAILURE` | alive |
| a node, at a later tick | yes | once, `(FAILURE, false)` | aborted, `FAILURE` | alive |
| `onLoopAfterTick()` | yes | once, `(FAILURE, false)` | aborted, `FAILURE` | alive |
| `onLoopFeedback()` | yes | once, `(FAILURE, false)` | aborted, `FAILURE` | alive |
| a node's halt, after a cancel | **the nodes after it: no** | once, `(FAILURE, false)` | aborted, not cancelled | alive |
| a node's halt, after a stop by `onLoopAfterTick()` | **the nodes after it: no** | once, `(FAILURE, false)` | aborted | alive |
| `onTreeCreated()` | nothing to halt | not called | aborted, `FAILURE` | alive |
| the hook itself, called for a cancel | yes | once, `(RUNNING, true)` | aborted | alive |
| the hook itself, called for a stop | yes | once, `(FAILURE, true)` | aborted | alive |
| the hook itself, at a normal end | ended | once | none | **ends**, `std::terminate` |
| the hook itself, after a node threw | yes | once | none | **ends**, `std::terminate` |
| a node, with `throw 42` | no | not called | none | **ends**, `std::terminate` |
| a node, with `%` in its message | no | not called | none | **ends**, `SIGSEGV` |

After every row that leaves the process alive, the next goal ran and succeeded. So the commit does what its message says: for an exception derived from `std::exception`, thrown after `onTreeCreated()` returns, the tree is halted and the hook is called once. The two flags, `tree_created` and `execution_completed`, are placed correctly: we found no path that calls the hook twice.

### What It Leaves Open

- **A halt that throws stops the halt.** `haltTree()` halts the nodes in order, and an exception leaves it at the first node that throws. The `catch` block calls `haltTree()` again, the same node throws again, and that second exception is logged. The nodes after it are never halted: a tree `Parallel(ThrowOnHalt, Forever)` ended with `Forever` not halted, and the hook was told the run had ended. The commit's inner `try` protects the hook, not the nodes. Verified. With `RosActionNode`, a halt throws in real use, see [A Halt at Any Moment](#a-halt-at-any-moment).
- **The hook is now called from a `catch` block, unprotected.** If it throws there, the exception leaves the goal thread. Verified. Upstream has the same hole at a normal end; the commit adds a second one.
- **Only `std::exception`.** Anything else thrown by a node ends the process. Verified.
- **The message of the exception is the format of the log.** `RCLCPP_ERROR(kLogger, action_result->return_message.c_str())`, 4 times in `execute()`: a `%` in the text of an exception, or in the message a hook returns, is read as a conversion. Upstream's line, but the `catch` block is where a text we do not write arrives. Verified, see issue 6.
- **`onTreeCreated()` that throws gets no hook.** The commit documents it. It is the defensible limit, but it puts a duty on the derived server: whatever `onTreeCreated()` sets and the hook clears must be set last. StepIt Commander does so, with `running_ = true` as its last line.
- **A shutdown is the one end of a run that still does not halt the tree**, see [Shutdown During a Run](#shutdown-during-a-run). It is not an exception, so the commit does not cover it, but it is the same promise: every end of a run halts and completes.

## Under a Server That Moves Motors

### A Halt at Any Moment

`RosActionNode::halt()` cancels the goal with the handle the node holds, `goal_handle_`. Three ways it goes wrong, all verified.

**The handle of the previous goal.** `tick()` resets four members when a node starts a new run, and not `goal_handle_`: it is replaced only when a tick takes the answer to the new goal. Until then, at least one tick period, longer when the server is slow to answer, a halt goes like this:

1. `cancelGoal()` sees a handle, so it does not wait for the answer to the new goal.
2. It calls `async_get_result()` with the handle of the previous goal, which the client has forgotten since its result arrived: `UnknownGoalHandleError`.
3. The new goal is never cancelled.

Measured on a tree `Repeat(2, Sleep 600 ms)`, halted as soon as the second goal was sent: `haltTree()` threw; the fake server received 2 goals and 0 cancels, and completed both. Through `TreeExecutionServer`, with a cancel of the `ExecuteTree` goal at that moment:

| The node | The `ExecuteTree` result | The action goal |
|---|---|---|
| `BT::RosActionNode` | aborted, `Behavior Tree exception:Goal handle is not known to this client.` | runs to its end, never cancelled |
| with a `halt()` that drops the exception, as StepIt Macro's | **cancelled**, `Action Server canceling and halting Behavior Tree` | runs to its end, never cancelled |

Dropping the exception in the user hides the symptom and keeps the defect: the client is told the tree was halted while the action goes on. The process survives in both cases; it is the destructor of a `BT::Tree` that ends it, see issue 9.

**The callbacks of a dead node.** The goal, feedback and result callbacks capture `this`, the tree node, and are kept by the action client. The client is shared through a static registry and outlives the node whenever the next tree has a node of the same action: `createTree()` runs before the old tree is destroyed, so the new node finds the client alive. A goal that was not cancelled therefore calls back into freed memory, from inside a tick of the new tree. Continuing the scenario above, with a new tree of the same node type ticked after the old one was destroyed: AddressSanitizer reports `heap-use-after-free` in `RosActionNode::logger()`, called by the goal response callback, on memory freed by the destruction of the old tree; the plain build ends with `SIGSEGV`.

**An action node at the root.** `RosActionNode::halt()` does not reset its status; its parent does. BehaviorTree.CPP 4.10's `haltTree()` halts the root, then visits every node and halts it again. An action node that is the root of its tree is still `RUNNING` at the second visit and cancels a goal that the first visit has just cancelled: `haltTree()` threw `UnknownGoalHandleError` after 10 ms against a healthy server, and the node stayed `RUNNING`. Under any control node it returned normally.

For StepIt Macro: the one action node its trees use, `Shoot`, sits in a loop of `FocusStack`, so the same instance runs once per picture, and a stop that falls in the window after a shot is requested fires that shot anyway. Starting a tree with a `Shoot` afterwards can then crash the server. Replacing the run with a tree that has no `Shoot`, as the stop button of its gamepad does, destroys the client and the pending callbacks with it. None of its 16 trees has an action node at the root.

### A Slow or Absent Action Server

`cancelGoal()` waits three times in turn on the thread that ticks the tree: for the answer to the goal, for the cancel, for the result, each up to `server_timeout`. Measured with a `server_timeout` of 1000 ms, on a `Sleep` goal of 4 s:

| The action server | `haltTree()` returns after | Cancels the server received | The goal |
|---|---|---|---|
| healthy | 10 ms | 1 | cancelled |
| hangs after accepting | 2000 ms | 0 | completes when the server comes back |
| destroyed after accepting | 2001 ms | 0 | none left |
| answers the goal after 2.5 s | 1002 ms | 0 | **accepted later, runs to its end** |
| answers after 0.8 s, then hangs | 2801 ms | 0 | runs to its end |
| refuses the cancel | 1000 ms | 1, refused | runs to its end |

Two findings in it. The halt blocks for 2 to 3 times `server_timeout`, during which the thread halts no other node, ticks nothing and, in a server, keeps the next goal waiting. And in four rows of six the halt returns normally, the node is `IDLE`, and the goal still runs: a halt reports nothing, so nobody learns that the cancel failed. Each of those goals keeps callbacks to the node, as above.

StepIt Commander sets `ros_plugins_timeout` to 2000 ms: a stop that meets a hung action server takes 4 s to 5.6 s before the next tree starts.

Two more, from reading. There is no timeout on the result: a server that dies after accepting leaves the node `RUNNING` until something halts it. And the creation of a client waits up to `wait_for_server_timeout` inside `createTree()`, once per client whose server is down.

### A Second Goal

`handle_accepted()` joins the thread of the previous goal on the thread that called it, an executor thread, inside the default callback group of the node, which is mutually exclusive. Verified with a `MultiThreadedExecutor`: with one tree running and a second goal accepted, a cancel of the running goal got no answer in 3 s, and neither did `get_loaded_trees`. More threads do not help: the group is busy. The parameter services of the node are in the same group.

A server that makes the running tree end from `onGoalReceived()`, as StepIt Commander does by default, shortens the wait to one tick, plus the halt: with the table above, up to 3 times `server_timeout` during which the server answers no cancel. A server that does not, StepIt Commander with `preempt` false included, cannot cancel a running tree once a second goal waits; if that tree never ends, the node is deaf for good.

### Shutdown During a Run

Verified, with the lines of StepIt Commander's `main()` and a `SIGINT` while a tree with an action goal and a second running node was ticked:

1. `spin()` returns.
2. The loop of `execute()` sees `rclcpp::ok()` false and leaves without halting the tree: 0 halts, 0 cancels at the action server.
3. `onTreeExecutionCompleted(RUNNING, false)` is called.
4. `goal_handle->abort()` throws `std::runtime_error`, `Asked to publish result for goal that does not exist`, outside any `try`: `std::terminate`, exit code 134.

An idle server that has run one goal ends the same way, in the destructor of the unjoined `std::thread`: `terminate called without an active exception`. So a server built on this class never exits cleanly once it has run a goal, and never cancels the goals of its nodes when it is stopped. When the whole system stops together, the other nodes stop their own hardware; a server restarted alone leaves a running action goal with no owner.

## Structure and Dependencies

Counted from the build files: `btcpp_ros2_samples` links `behaviortree_ros2`, which links `btcpp_ros2_interfaces`, `behaviortree_cpp`, `rclcpp_action`, `ament_index_cpp` and the generated `bt_executor_parameters`. No cycle, no include against that direction.

| Question | Judgement |
|---|---|
| Boundaries | Kept. The server knows no node wrapper; the wrappers know no server. `bt_utils.hpp` is public and useful to a derived server, which is how `GetDirectoryPath()` of issue 8 reaches StepIt Commander's own code. |
| Interfaces | `TreeExecutionServer` is extended by six virtual hooks: a small, clear surface. Its weakness is that they run on two threads, [The Hooks](ARCHITECTURE.md#the-hooks), and the class gives no means to pass a request from one to the other; each derived server builds its own flag. `factory()` hands out the factory with no lock, to both threads. |
| Single responsibility | `RosActionNode<T>` is a client registry, a state machine and a cancel protocol in one header template of 579 lines, with its state in 6 private members. That is why a user cannot repair it by deriving from it. |
| Open for extension | `RosActionNode::halt()` is virtual, `cancelGoal()` is not, and `goal_handle_` and `future_goal_handle_` are private: a derived node can wrap the halt, not correct it. |
| Shared state | Three static registries, one per wrapper template and message type, keyed by node and name, and one static mutex per template. They are the reason a callback outlives its node. |
| Duplication | The constructor, `createClient()` and the registry are written three times, in the action, service and subscriber nodes, 60 to 80 lines each, and have already diverged: `RosActionNode::tick()` locks the mutex around its executor, `RosServiceNode::tick()` does not. |

## Keeping the Fork

**Keeping it is right.** `behaviortree_ros2` has no Debian package, so a user builds it from source in any case, and the fork's commit is needed by any server that reports the end of its runs.

**Its neighbourhood is not what the architecture document assumes.** Read on 2026-11-15:

- Upstream's last commit is of 2025-11-25. Its only branches are `humble`, `bug_fix/registry` and `issue_76`.
- The fork's pull request, 135, is **closed and not merged**: closed by its author 10 minutes after it was opened, with one comment, "partial". "Drop the fork once upstream has merged its pull request 135" has nothing to wait for.
- The defect of [A Halt at Any Moment](#a-halt-at-any-moment) is known upstream and unresolved: issues 18 and 110 are open, and so are two pull requests on it, 129 and 134.

So the rule "every commit of the fork is a pull request upstream, and a larger weakness is worked around in the users" does not hold up. The fork has no open pull request, upstream is not merging, and the weakness that matters cannot be worked around in a user. We would turn the rule round: the fork is the maintained copy, each fix is one commit small enough to offer upstream, and it is offered, but the users do not wait for it.

What should go upstream, in the order of [Recommendations](#recommendations): the commit of pull request 135, reopened with the two hooks protected; the reset of the goal handle; the format of the four log lines; `npos` in `GetDirectoryPath()`.

## Testability and Tests

The repository has no test. The fork's change has one, in StepIt Commander: `ATreeThatThrowsEndsItsRun` runs a tree whose `Script` node throws after 300 ms, and checks an aborted goal, `FAILURE`, and the end of the run published. It covers the second row of [Every Path Through execute()](#every-path-through-execute), and none of the others. Nothing covers `RosActionNode`: StepIt Macro's tests run against fakes that answer at once.

Nothing stands in the way of tests here. Everything this review ran is a single process: the server on its own executor, the fake action server on a second ROS context, the trees from text, a private package index for the plugins. Three seams are missing, and each cost a workaround:

- **The server cannot be destroyed** once it has run a goal, issue 5: a test must leak it, or leave with `_Exit`. StepIt Commander's tests keep every server alive until the process ends for that reason.
- **The moment a goal is answered cannot be chosen** from outside the node. The fake server delays its answer instead, which works.
- **A halt reports nothing**: a test learns whether the goal was cancelled only from the server's side.

## Implementation Issues

Ordered by severity. "Safety" says what the issue means for a server whose trees move hardware, and for StepIt Macro.

| # | Issue | Where | Evidence | Safety |
|---|---|---|---|---|
| 1 | A node halted before its second goal is answered cancels its previous goal: the new goal is never cancelled. | `RosActionNode::tick()`, the `IDLE` branch, and `cancelGoal()` | verified | **Yes**: an action goes on after a halt. In StepIt Macro, one more shot; no motion. |
| 2 | The callbacks of a goal capture the tree node and outlive it through the shared client: use after free, at a tick of the next tree. | the three lambdas of `RosActionNode::tick()` | verified, AddressSanitizer | **Yes, indirectly**: the server crashes, and a dead server serves no stop. |
| 3 | A halt blocks the tick thread for 2 to 3 times `server_timeout`, then returns as if it had cancelled. A goal answered late is accepted and never cancelled. | `RosActionNode::cancelGoal()` | verified | **Yes**: a stop is late by up to 6 s at 2000 ms, and may not stop the action. |
| 4 | A second goal blocks an executor thread and the node's default callback group until the running tree ends: no cancel, no `get_loaded_trees`. | `TreeExecutionServer::handle_accepted()` | verified | **Yes** for a server that does not end the running tree itself. Bounded by issue 3 for one that does. |
| 5 | Shutdown during a run: the tree is not halted, no action goal is cancelled, and the process ends in `std::terminate`. Idle after one goal: the same end. | `execute()`, `~TreeExecutionServer()` | verified | Low when everything stops together; a goal left running when the server alone is stopped. |
| 6 | The return message is passed as the format of the log: a `%` in the text of an exception reads the stack. | `execute()`, 4 calls of `RCLCPP_*` | verified, `SIGSEGV` | **Yes, indirectly**, as 2. Also a security issue: whoever may send a goal may choose the text, e.g. through a port value. |
| 7 | Exceptions that leave the goal thread: any type not derived from `std::exception`; `onTreeExecutionCompleted()` at a normal end or in the `catch` block; a registration at the start of a goal, e.g. `registerNodesIntoFactory()` registering an ID twice. | `execute()` | verified, 4 cases | **Yes, indirectly**, as 2. |
| 8 | A folder parameter without `/` throws `std::length_error`: the test is against `size()`, not `npos`. The server dies when it starts. | `GetDirectoryPath()` | verified | No: at start only. |
| 9 | A halt that throws leaves the nodes after it running; and a `BT::Tree` destroyed while such a node is `RUNNING` ends the process, since its destructor halts. | `execute()`; `RosActionNode::halt()` | verified | **Yes** where the node left running moves something. |
| 10 | An action node at the root of a tree is halted twice and throws against a healthy server. | `RosActionNode::halt()`, which does not reset its status | verified | Low: the goal is cancelled first. None of StepIt Macro's trees. |
| 11 | Two running nodes of one action share a client; a tick of one takes the answers of the other. The result of a node that has not yet taken its handle dereferences a null `goal_handle_`. | the result callback, `bt_action_node.hpp` line 423 | verified, `SIGSEGV`, 3 runs of 3 | As 2, for trees with two such nodes in a `Parallel`. None in StepIt Macro. |
| 12 | The same line with a handle of a previous run drops the result, and the node stays `RUNNING` for ever. Feedback has no check at all: that of a goal left running is given to the node's next run. | the result and feedback callbacks | from reading | No. |
| 13 | A registration runs again before the next goal after any parameter is declared or set on the node, with `generate_parameter_library` 0.7.6. It logs `Failed to load ROS Plugin` for each plugin, and forgets the trees a derived server registered through `factory()`. | `execute()`, `executeRegistration()`, `LoadPlugin()` | verified | No. |
| 14 | The factory is used from two threads with no lock: a registration on the goal thread against `get_loaded_trees`, or against a derived server that reads or fills `factory()` in `onGoalReceived()`. | `execute()`, the service, `factory()` | from reading | No: a rare crash at the start of a goal. |
| 15 | No timeout on the result of an action. | `RosActionNode::tick()` | from reading | No: the tree waits, and can be halted. |
| 16 | After a tick longer than the period, the loop ticks without sleeping until it has caught up: `loop_deadline += period`. | `execute()` | from reading | No. |
| 17 | `RosTopicSubNode` subscribes with the default quality of service: a latched message published earlier is never received. | `SubscriberInstance` | verified, 0 of 1 | No. StepIt Macro does not use it. |

## Suspicions the Tests Disproved

- **That the hook could be called twice**, e.g. when a halt throws inside a stop and the `catch` block completes the run again. It is called once on every path.
- **That an exception in a halt ends the process under `TreeExecutionServer`.** It does not, at this commit: both halts are inside a `try`, the run ends aborted, and the next goals run. A tree is replaced by a move assignment, which does not halt. Only the destruction of a `BT::Tree` object does, issue 9.
- **That one action node alone can meet the null handle of issue 11.** 60 runs of a single node against a server that answers at once, ticked every 1, 10 and 200 ms: none failed. It takes two nodes on one client.
- **That a cancel which throws leaves the goal of the `ExecuteTree` action without an end.** It is aborted, and the client gets a result.

## What the Documents Say and the Tests Do Not

In [docs/ARCHITECTURE.md](ARCHITECTURE.md):

- **"A node that is halted when its goal has just ended, its result taken by the client but not yet turned into a status by a tick."** One node cannot be in that state: the tick that takes the result turns it into a status. The three real cases are the handle of the previous run, an action node at the root, and two nodes on one client.
- **"When ROS shuts down during a run ... the goal is aborted."** `abort()` throws and the process ends, [Shutdown During a Run](#shutdown-during-a-run).
- **"The same commit is the head of upstream's pull request 135"**, and "once upstream has merged its pull request 135": the pull request is closed, unmerged.
- **"The release in rosdistro for Jazzy ... 0.7.7."** The image we measured has 0.7.6, which registers again on every parameter set or declared. We compared the two tags of `generate_parameter_library`: 0.7.7 stamps only when one of the server's own parameters is set. An upgrade of that package changes the behaviour with no change here.
- **"The executor thread that waits serves nothing else of the action server."** It is wider: nothing else of the node's default callback group.

In the notes of StepIt Macro: halting a node whose goal has just ended "would end the commander's process". The exception is real, in the first case above; at this commit it does not end the process, and dropping it leaves the new goal running.

## Recommendations

Ordered by value against effort; each is one pull request, in the fork.

1. **Reset `goal_handle_` when a node starts a run**, one line in the `IDLE` branch of `tick()`, and test the handle for null in the result callback. `cancelGoal()` then waits for the answer to the new goal and cancels the right one. Fixes 1 and 12, and the crash of 11. Trade-off: none; the users' own `halt()` becomes dead code and can go.
2. **Print the message with `"%s"`** in the four log calls of `execute()`. Fixes 6. No trade-off.
3. **Compare with `npos`** in `GetDirectoryPath()`. Fixes 8. No trade-off.
4. **Keep the callbacks away from a dead node**: capture a shared state the node owns, held weakly, in place of `this`. Fixes 2 and the feedback of 12. Trade-off: about 30 lines in the node, and the result of a goal nobody waits for is dropped silently.
5. **Close `execute()`**: catch everything; protect both calls of the hook; move the registration inside the `try`; halt the tree when `rclcpp::ok()` turns false; do not let `abort()` throw out of the thread; halt the remaining nodes when one halt throws. Join the thread in the destructor. Fixes 5, 7 and 9. Trade-off: a hook that throws is logged and swallowed, and a destructor that waits for a halt, see the next one.
6. **Bound the halt**: one deadline for the three waits of `cancelGoal()`, a log when the cancel was not confirmed, and `resetStatus()` at the end of `halt()`. Fixes 10 and the length of 3. Trade-off: a halt returns before the server confirms; the goal that could not be cancelled is still there, now reported. Stopping it for good needs the node to cancel when the late answer arrives, which is the larger change.
7. **Add a test package**, `behaviortree_ros2_tests`, from the scenarios of this review: the table of [Every Path Through execute()](#every-path-through-execute), the halt before the second answer, the halt against a hung server, the second goal, the shutdown. Each recommendation above then lands with the test that fails without it. Trade-off: the first tests in a repository that has none, and a `colcon test` to add to its workflow.
8. **Do not join on the executor thread** in `handle_accepted()`: give the class a way to end the running tree, and hand the join to the new goal's thread. Fixes 4. Trade-off: the largest change, and it moves upstream's behaviour, so it should follow the tests of 7.
9. **Record the version of `generate_parameter_library`** the behaviour of 13 depends on, with a test in the user that fails when a declared parameter stops, or starts, causing a registration. Trade-off: a test of somebody else's package.
