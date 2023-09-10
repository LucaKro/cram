(in-package :ob-plans)

(def-fact-group opening-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (open-bottle ?resolved-action-designator))
    (spec:property ?action-designator (:type :opening-bottle))
    (spec:property ?action-designator (:object ?object))
    
    (-> (spec:property ?action-designator (:object-cap ?object-cap))
        (spec:property ?action-designator (:object-cap ?object-cap))
        (lisp-fun get-object-lid ?object ?object-cap))

    ;; (and (format "WARNING: Please specify with an arm which ooooooooooooooooooooooooooooo")
    ;;      (fail))
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
    
 
    (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
   
    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (and (lisp-fun man-int:get-action-grasps ?object-type ?bottle-hand ?object-transform ?grasps)
             (member ?grasp ?grasps)))
    
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-cap-type ?open-hand-gripper-opening)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?bottle-hand-gripper-opening)
    
    (and (lisp-fun man-int:get-action-trajectory :open-bottle ?open-hand ?grasp T ?object-cap ?pose)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :open
                   ?open-hand-open-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :pre-open
                   ?open-hand-pre-open-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :approach
                   ?open-hand-approach-poses)
         (lisp-fun man-int:get-traj-poses-by-label ?pose :grasping
                   ?open-hand-grasp-poses))
    ;; (equal ?open-hand-open-poses nil)
    ;; (equal ?open-hand-pre-open-poses nil)
    ;; (equal ?open-hand-approach-poses nil)
    ;; (equal ?open-hand-grasp-poses nil)
    ;; (equal ?bottle-hand-grasp-poses nil)
    ;; (equal ?bottle-hand-approach-poses nil)

    (and (lisp-fun man-int:get-action-trajectory :hold-bottle ?bottle-hand ?grasp T ?object
                   ?bottle-hand-trajectory)
         (lisp-fun man-int:get-traj-poses-by-label ?bottle-hand-trajectory :approach
                   ?bottle-hand-approach-poses)
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
                               (:collision-mode ?collision-mode)
                               (:bottle-hand-gripper-opening ?bottle-hand-gripper-opening)
                               (:open-hand-gripper-opening ?open-hand-gripper-opening)
                               (:effort ?effort)
                               (:grasp ?grasp)
                               (:bottle-hand ?bottle-hand)
                               (:open-hand ?open-hand)
                               (:bottle-hand-approach-poses ?bottle-hand-approach-poses)
                               (:bottle-hand-grasp-poses ?bottle-hand-grasp-poses)
                               (:open-hand-approach-poses ?open-hand-approach-poses)
                               (:open-hand-grasp-poses ?open-hand-grasp-poses)
                               (:open-hand-pre-open-poses ?open-hand-pre-open-poses)
                               (:open-hand-open-poses ?open-hand-open-poses))
                      ?resolved-action-designator)))
