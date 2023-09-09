(in-package :ob-plans)

(def-fact-group my-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (2hand-bottle-func ?resolved-action-designator))
    (spec:property ?action-designator (:type :2hand-bottle))
    (spec:property ?action-designator (:object ?bottle))
    (-> (spec:property ?action-designator (:object-cap ?object-cap))
        (spec:property ?action-designator (:object-cap ?object-cap))
        (lisp-fun get-object-lid ?bottle ?object-cap))

    (spec:property ?object-cap (:type ?object-cap-type))

    (desig:current-designator ?bottle ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    ;; (spec:property ?current-object-desig (:name ?object-name))
    
    ;; (-> (spec:property ?action-designator (:arm ?arm))
    ;;     (true)
    ;;     (man-int:robot-free-hand ?_ ?arm))
    
 
    (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
   
    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (and (lisp-fun man-int:get-action-grasps ?object-type '(:right) ?object-transform ?grasps)
             (member ?grasp ?grasps)))
    
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-cap-type ?left-gripper-opening)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?right-gripper-opening)
    
    (and (lisp-fun man-int:get-action-trajectory :open-bottle :left ?grasp T ?object-cap ?pose)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :open
                   ?left-open-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :pre-open
                   ?left-pre-open-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :approach
                   ?left-approach-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :grasping
                   ?left-grasp-poses))
    ;; (equal ?left-open-poses nil)
    ;; (equal ?left-pre-open-poses nil)
    ;; (equal ?left-approach-poses nil)
    ;; (equal ?left-grasp-poses nil)
    ;; (equal ?right-grasp-poses nil)
    ;; (equal ?right-approach-poses nil)

    (and (lisp-fun man-int:get-action-trajectory :open-bottle :right ?grasp T ?bottle
                   ?right-trajectory)
         (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :approach
                   ?right-approach-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
                   ?right-grasp-poses))

    (-> (desig:desig-prop ?action-designator (:collision-mode ?collision-mode))
        (true)
        (equal ?collision-mode NIL))

    ;;(lisp-fun man-int:get-action-gripping-effort :milk ?effort)

    ;; (and (format "WARNING: Please specify with an arm which ooooooooooooooooooooooooooooo")
    ;;      (fail))
    
    (desig:designator :action ((:type :2hand-bottle)
                               (:object ?current-object-desig)
                               (:object-cap ?object-cap)
                               (:collision-mode ?collision-mode)
                               (:right-gripper-opening ?right-gripper-opening)
                               (:left-gripper-opening ?left-gripper-opening)
                               (:effort ?effort)
                               (:grasp ?grasp)
                               (:right-approach-poses ?right-approach-poses)
                               (:right-grasp-poses ?right-grasp-poses)
                               (:left-approach-poses ?left-approach-poses)
                               (:left-grasp-poses ?left-grasp-poses)
                               (:left-pre-open-poses ?left-pre-open-poses)
                               (:left-open-poses ?left-open-poses))
                      ?resolved-action-designator)))
