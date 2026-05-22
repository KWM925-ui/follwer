function fsm = updateFsm(currentState, context, cfg)
%updateFsm Update the deterministic recovery state machine.

    persistent lostFrames visibleFrames searchFrames holdFrames

    if isfield(context, "reset") && context.reset
        lostFrames = 0;
        visibleFrames = 0;
        searchFrames = 0;
        holdFrames = 0;
        fsm.state = "FOLLOW";
        return
    end

    if isempty(lostFrames)
        lostFrames = 0;
        visibleFrames = 0;
        searchFrames = 0;
        holdFrames = 0;
    end

    plannerFailureActive = (~isfield(cfg, "usePlannerFeedback") || cfg.usePlannerFeedback) ...
        && ~context.plannerHealthy;

    if isfield(cfg, "useFsmRecovery") && ~cfg.useFsmRecovery
        if context.covarianceTrace > cfg.covarianceFailsafeThreshold
            state = "FAILSAFE";
        elseif plannerFailureActive
            state = "HOLD_SAFE";
        else
            state = "FOLLOW";
        end

        fsm.state = state;
        fsm.lostFrames = lostFrames;
        fsm.visibleFrames = visibleFrames;
        fsm.searchFrames = searchFrames;
        fsm.holdFrames = holdFrames;
        return
    end

    if context.visible
        visibleFrames = visibleFrames + 1;
        lostFrames = 0;
    else
        lostFrames = lostFrames + 1;
        visibleFrames = 0;
    end

    state = string(currentState);

    if context.covarianceTrace > cfg.covarianceFailsafeThreshold
        state = "FAILSAFE";
    elseif plannerFailureActive
        state = "HOLD_SAFE";
    elseif ~context.hasSafeCandidate
        state = "HOLD_SAFE";
    else
        switch state
            case "FOLLOW"
                if ~context.visible
                    state = "PREDICT_HOLD";
                end

            case "PREDICT_HOLD"
                if context.visible && visibleFrames >= cfg.reacquireFrames
                    state = "REACQUIRE";
                elseif lostFrames > cfg.predictHoldFrames || context.covarianceTrace > cfg.covarianceHoldThreshold
                    state = "SEARCH_SAFE_VIEWPOINT";
                end

            case "SEARCH_SAFE_VIEWPOINT"
                searchFrames = searchFrames + 1;
                if context.visible
                    state = "REACQUIRE";
                    searchFrames = 0;
                elseif searchFrames > cfg.searchFramesToFailsafe
                    state = "FAILSAFE";
                end

            case "REACQUIRE"
                if context.visible && visibleFrames >= cfg.reacquireFrames
                    state = "FOLLOW";
                elseif ~context.visible
                    state = "PREDICT_HOLD";
                end

            case "HOLD_SAFE"
                holdFrames = holdFrames + 1;
                if context.hasSafeCandidate && context.visible
                    state = "REACQUIRE";
                    holdFrames = 0;
                elseif holdFrames > cfg.safeHoldFramesToFailsafe
                    state = "FAILSAFE";
                end

            case "FAILSAFE"
                state = "FAILSAFE";

            otherwise
                state = "FOLLOW";
        end
    end

    if state ~= "SEARCH_SAFE_VIEWPOINT"
        searchFrames = 0;
    end
    if state ~= "HOLD_SAFE"
        holdFrames = 0;
    end

    fsm.state = state;
    fsm.lostFrames = lostFrames;
    fsm.visibleFrames = visibleFrames;
    fsm.searchFrames = searchFrames;
    fsm.holdFrames = holdFrames;
end
