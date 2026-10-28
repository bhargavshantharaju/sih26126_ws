# SESSION ACTION PLAN
## Immediate Focus: Fix Sim Stability & Robot Dynamics

Based on the comprehensive analysis, this session will focus on:

## COMPLETED ✅

1. **Created SYSTEM_REDESIGN_PLAN.md** - Comprehensive architecture redesign
2. **Created minimal_stable.world** - Simple test environment
3. **Created test_minimal.launch.py** - Test harness

## PRIORITY ACTIONS FOR THIS SESSION

### 1. Test Minimal World Performance (30 min)
- Build and launch minimal world
- Measure sim/wall time ratio
- Check for clock jumps  
- Verify >95% real-time

### 2. Test Robot Rotation (20 min)
- Publish test cmd_vel commands
- Measure actual vs commanded rotation
- Verify rotation direction
- Document findings

### 3. Fix Robot Dynamics If Needed (30 min)
- If rotation rate wrong: adjust inertia/friction
- If rotation direction wrong: investigate DiffDrive or planner sign
- Re-test until robot responds correctly

### 4. Launch SLAM in Minimal World (20 min)
- Start slam_toolbox
- Verify map→odom TF
- Verify no message drops
- Confirm stable operation

### 5. Document Findings (10 min)
- What was fixed
- What works now
- Next session priorities
- Evidence/measurements

## WHAT NOT TO DO THIS SESSION

- ❌ Don't try to complete full autonomy
- ❌ Don't modify complex planning logic yet
- ❌ Don't add frontier scoring yet
- ❌ Don't test full stack yet

## SUCCESS CRITERIA FOR THIS SESSION

1. Minimal world runs at >95% real-time with stable clock
2. Robot rotation dynamics understood and documented
3. If rotation is broken, root cause identified
4. SLAM can run stably in minimal world
5. Clear path forward documented

## DEFERRED TO NEXT SESSIONS

- Frontier utility scoring
- Full navigation testing
- Two-cycle autonomy validation
- Dashboard improvements
- Documentation polish

---

**Philosophy**: Build stable foundation first. Can't test autonomy if sim is unstable or robot doesn't move correctly.

