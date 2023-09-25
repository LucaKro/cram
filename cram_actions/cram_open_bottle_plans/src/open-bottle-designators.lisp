(in-package :ob-plans)

(def-fact-group opening-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (open-bottle ?resolved-action-designator))
    (spec:property ?action-designator (:type :opening-bottle))
    (spec:property ?action-designator (:object ?object))
    
    (-> (spec:property ?action-designator (:object-cap ?object-cap))
        (true)
        (lisp-fun get-object-lid ?object ?object-cap))

    (-> (spec:property ?action-designator (:with-tool ?tool))
        (true)
        (equal ?tool nil))

    (spec:property ?object-cap (:type ?object-cap-type))

    (desig:current-designator ?object ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    ;; (spec:property ?current-object-desig (:name ?object-name))
    
    (-> (spec:property ?action-designator (:arm ?bottle-hand))
        (true)
        (man-int:robot-free-hand ?_ ?bottle-hand))
    
    (-> (equal ?bottle-hand :right)
        (equal ?open-hand :left)
        (equal ?open-hand :right))
    ;; (equal ?bottle-hand :left)
    
 
    (lisp-fun man-int:get-object-transform ?current-object-desig ?bottle-transform)
    (lisp-fun man-int:get-object-transform ?object-cap ?cap-transform)

    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    ;;(lisp-fun man-int:calculate-object-faces ?bottle-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?bottle-rotationally-symmetric t)
        (equal ?bottle-rotationally-symmetric nil))

    (lisp-fun man-int:calculate-object-faces ?bottle-transform (?facing-robot-face ?bottom-face))
    
    (-> (man-int:object-rotationally-symmetric ?object-cap-type)
        (equal ?cap-rotationally-symmetric t)
        (equal ?cap-rotationally-symmetric nil))
    
    (and (lisp-fun man-int:get-action-grasps ?object-type ?bottle-hand ?bottle-transform ?bottle-hand-grasps)
         (member ?bottle-hand-grasp ?bottle-hand-grasps))
    
    (and (lisp-fun man-int:get-action-grasps ?object-cap-type ?open-hand ?cap-transform ?open-hand-grasps)
         (member ?open-hand-grasp ?open-hand-grasps))
    
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripping-effort ?object-cap-type ?cap-effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-cap-type ?open-hand-gripper-opening)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?bottle-hand-gripper-opening)
    
    (and (lisp-fun man-int:get-action-trajectory :open-bottle ?open-hand ?open-hand-grasp T ?object-cap :tool ?tool
                   ?open-hand-trajectory)
         (lisp-fun man-int:get-traj-poses-by-label ?open-hand-trajectory :reach
                   ?open-hand-reach-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?open-hand-trajectory :grasping
                   ?open-hand-grasp-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?open-hand-trajectory :pre-open
                   ?open-hand-pre-open-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?open-hand-trajectory :open
                   ?open-hand-open-poses)
         )
    ;; (equal ?open-hand-open-poses nil)
    ;; (equal ?open-hand-pre-open-poses nil)
    ;; (equal ?open-hand-reach-poses nil)
    ;; (equal ?open-hand-grasp-poses nil)
    ;; (equal ?bottle-hand-grasp-poses nil)
    ;; (equal ?bottle-hand-reach-poses nil)

    (and (lisp-fun man-int:get-action-trajectory :hold-bottle ?bottle-hand ?facing-robot-face T ?object
                   ?bottle-hand-trajectory)
         (lisp-fun man-int:get-traj-poses-by-label ?bottle-hand-trajectory :reach
                   ?bottle-hand-reach-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?bottle-hand-trajectory :grasping
                   ?bottle-hand-grasp-poses))

    
    (-> (desig:desig-prop ?action-designator (:collision-mode ?collision-mode))
        (true)
        (equal ?collision-mode NIL))

    ;;(lisp-fun man-int:get-action-gripping-effort :milk ?effort)

    ;; (and (format "WARNING: Please specify with an arm which ooooooooooooooooooooooooooooo")
    ;;      (fail))
    
    (desig:designator :action ((:type :opening-bottle)
                               (:object ?current-object-desig)
                               (:object-cap ?object-cap)
                               (:bottle-hand-gripper-opening ?bottle-hand-gripper-opening)
                               (:open-hand-gripper-opening ?open-hand-gripper-opening)
                               (:effort ?effort)
                               (:tool ?tool)
                               (:bottle-hand ?bottle-hand)
                               (:bottle-hand-grasp ?bottle-hand-grasp)
                               (:open-hand ?open-hand)
                               (:open-hand-grasp ?open-hand-grasp)
                               (:bottle-hand-reach-poses ?bottle-hand-reach-poses)
                               (:bottle-hand-grasp-poses ?bottle-hand-grasp-poses)
                               (:open-hand-reach-poses ?open-hand-reach-poses)
                               (:open-hand-grasp-poses ?open-hand-grasp-poses)
                               (:open-hand-pre-open-poses ?open-hand-pre-open-poses)
                               (:open-hand-open-poses ?open-hand-open-poses))
                      ?resolved-action-designator)))
