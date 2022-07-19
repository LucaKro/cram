(in-package :demo)

(def-fact-group open-bottle-designators (desig:action-grounding)
  
  (<- (desig:action-grounding ?action-designator (find-and-open-bottle
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :finding-and-opening-bottle))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; context
    (once (or (spec:property ?action-designator (:context ?context))
              (equal ?context NIL)))
    ;; search location
    (-> (and (spec:property ?object-designator (:location ?some-search-loc-desig))
             (not (equal ?some-search-loc-desig NIL)))
        (and (desig:current-designator ?some-search-loc-desig
                                       ?search-location-designator)
             (equal ?object-designator-with-location ?object-designator))
        (and (spec:property ?object-designator (:type ?object-type))
             (rob-int:environment-name ?environment)
             (lisp-fun man-int:get-object-likely-location
                       ?object-type ?environment nil ?context
                       ?search-location-designator)
             (equal ?new-props ((:location ?search-location-designator)))
             (lisp-fun desig:extend-designator-properties
                       ?object-designator ?new-props
                       ?object-designator-with-location)
             (lisp-pred desig:equate
                        ?object-designator ?object-designator-with-location)))
    ;; search location robot base
    (-> (desig:desig-prop ?action-designator
                          (:search-robot-location ?some-s-robot-loc-desig))
        (desig:current-designator ?some-s-robot-loc-desig
                                  ?search-robot-location-designator)
        (equal ?search-robot-location-designator NIL))
    ;; fetch location robot base
    (-> (desig:desig-prop ?action-designator
                          (:open-bottle-robot-location ?some-f-robot-loc-desig))
        (desig:current-designator ?some-f-robot-loc-desig
                                  ?open-bottle-robot-location-designator)
        (equal ?open-bottle-robot-location-designator NIL))
    ;; arms
    (-> (spec:property ?action-designator (:arms ?arms))
        (true)
        (equal ?arms NIL))
    ;; grasps
    (-> (spec:property ?action-designator (:grasps ?grasps))
        (true)
        (equal ?grasps NIL))
    ;; deliver location
    (-> (and (spec:property ?action-designator
                            (:target ?some-delivering-location-designator))
             (not (equal ?some-delivering-location-designator NIL)))
        (desig:current-designator ?some-delivering-location-designator
                                  ?delivering-location-designator)
        (and (spec:property ?object-designator (:type ?object-type))
             (rob-int:environment-name ?environment)
             (lisp-fun man-int:get-object-destination
                       ?object-type ?environment nil ?context
                       ?delivering-location-designator)))
    ;; deliver location robot base
    (-> (desig:desig-prop ?action-designator
                          (:deliver-robot-location ?some-d-robot-loc-desig))
        (desig:current-designator ?some-d-robot-loc-desig
                                  ?deliver-robot-location-designator)
        (equal ?deliver-robot-location-designator NIL))
    ;; resulting action desig
    (desig:designator
     :action
     ((:type :finding-and-opening-bottle)
      (:object ?object-designator-with-location)
      (:context ?context)
      (:search-location ?search-location-designator)
      (:search-robot-location ?search-robot-location-designator)
      (:open-bottle-robot-location ?open-bottle-robot-location-designator)
      (:arms ?arms)
      (:grasps ?grasps)
      (:deliver-location ?delivering-location-designator)
      (:deliver-robot-location ?deliver-robot-location-designator))
     ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (open-bottle ?resolved-action-designator))
    (spec:property ?action-designator (:type :opening-bottle))
    (rob-int:robot ?robot)
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; arms
    (-> (spec:property ?action-designator (:arms ?arms))
        (true)
        ;; (equal ?arms NIL)
        (setof ?arm (man-int:robot-free-hand ?robot ?arm) ?arms))
    ;; grasps
    (-> (spec:property ?action-designator (:grasps ?grasps))
        (true)
        ;; we do not ask for grasps because they are arm-specific!
        ;; therefore, projection reasoning is essential for these plans
        ;; (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform
        ;;           ?grasps)
        (equal ?grasps NIL))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location
                                                      ?some-location-designator))
                   (desig:current-designator ?some-location-designator
                                             ?robot-location-designator))
              (desig:designator :location ((:reachable-for ?robot)
                                           ;; ?arm is not available because we're sampling
                                           ;; (:arm ?arm)
                                           (:object ?object-designator)
                                           ;; have to add the visibility
                                           ;; constraint as he reperceives
                                           ;; each time before grasping
                                           (:visible-for ?robot))
                                ?robot-location-designator)))
    ;; if the object is in the hand or its reference object is in the hand
    ;; we need to bring the hand closer to the other hand, e.g., bring to front
    (-> (man-int:object-or-its-reference-in-hand ?object-designator ?object-hand)
        (equal ?object-in-hand T)
        (and (equal ?object-in-hand NIL)
             (equal ?object-hand NIL)))
    ;; look-location
    (once (or (and (spec:property ?action-designator (:look-location
                                                      ?some-look-loc-desig))
                   (desig:current-designator ?some-look-loc-desig
                                             ?look-location-designator))
              (-> (or (equal ?object-in-hand NIL) (equal ?object-hand NIL))
                  (desig:designator :location ((:of ?object-designator))
                                    ?look-location-designator)
                  (and (desig:designator :object ((:part-of ?robot)
                                                  (:link :robot-tool-frame)
                                                  (:which-link ?object-hand))
                                         ?object-hand-designator)
                       (desig:designator :location ((:of ?object-hand-designator))
                                         ?look-location-designator)))))
    ;; pick-up-action
    (once (or (desig:desig-prop ;; spec:property
               ?action-designator (:pick-up-action ?some-pick-up-action-designator))
              (equal ?some-pick-up-action-designator NIL)))
    ;; pick-up-action
    (once (or (desig:desig-prop ;; spec:property
               ?action-designator (:remove-cap-action ?some-remove-cap-action-designator))
              (equal ?some-remove-cap-action-designator NIL)))
    (desig:current-designator ?some-pick-up-action-designator ?pick-up-action-designator)
    (desig:current-designator ?some-remove-cap-action-designator ?remove-cap-action-designator)
    (desig:designator :action ((:type :opening-bottle)
                               (:object ?object-designator)
                               (:arms ?arms)
                               (:grasps ?grasps)
                               (:robot-location ?robot-location-designator)
                               (:look-location ?look-location-designator)
                               (:pick-up-action ?pick-up-action-designator)
                               (:remove-cap-action ?remove-cap-action-designator) 
                               (:object-in-hand ?object-in-hand)
                               (:object-hand ?object-hand))
                      ?resolved-action-designator))



  (<- (desig:action-grounding ?action-designator (remove-cap ?resolved-action-designator))
    (spec:property ?action-designator (:type :removing-cap))

    ;; extract info from ?action-designator
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))

    ;; get the arm for grasping by checking if it is specified for ?object-type
    (man-int:arms-for-object-type ?object-type ?arms-for-object)
    (-> (equal ?arms-for-object nil)
        (-> (spec:property ?action-designator (:arm ?arm))
            (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
                 (subset ?arm ?free-arms))
            (and (man-int:robot-free-hand ?_ ?free-arm)
                 (equal (?free-arm) ?arm)))
        (-> (spec:property ?action-designator (:arm ?arm))
            (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
                 (subset ?arm ?free-arms)
                 (man-int:check-arms-for-object-type ?arm ?object-type))
            (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
                 (man-int:check-arms-for-object-type ?free-arms ?object-type)
                 (equal ?arm ?free-arms))))
                 
    (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)
    ;; get the type of the picking location, because the trajectory
    ;; might be different depending on the location type
    (once (or (and (spec:property ?current-object-desig (:location ?obj-loc))
                   (desig:current-designator ?obj-loc ?curr-obj-loc)
                   (man-int:location-reference-object ?curr-obj-loc ?obj-loc-obj)
                   (desig:current-designator ?obj-loc-obj ?curr-obj-loc-obj)
                   (spec:property ?curr-obj-loc-obj (:type ?location-type)))
              (equal ?location-type NIL)))

    ;; calculate trajectory with given grasps
    (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)    
    (equal ?objects (?current-object-desig))
    (-> (member :left ?arm)
        (and (-> (spec:property ?action-designator (:left-grasp ?left-grasp))
                 (true)
                 (member ?left-grasp ?grasps))
             (lisp-fun man-int:get-action-trajectory :removing-cap :left
                       ?grasp ?location-type ?objects
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grasping
                       ?left-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :openlid
                       ?left-openlid-poses))
        (and (equal ?left-grasp NIL)
             (equal ?left-reach-poses NIL)
             (equal ?left-grasp-poses NIL)
             (equal ?left-openlid-poses NIL)))

    (-> (member :right ?arm)
        (and  (-> (spec:property ?action-designator (:right-grasp ?right-grasp))
                  (true)
                  (member ?right-grasp ?grasps))
              (lisp-fun man-int:get-action-trajectory :removing-cap :right
                        ?grasp ?location-type ?objects
                        ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
                       ?right-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :openlid
                       ?right-openlid-poses))
        (and (equal ?right-grasp NIL)
             (equal ?right-reach-poses NIL)
             (equal ?right-grasp-poses NIL)
             (equal ?right-remove-cap-poses NIL)))

    (once (or (lisp-pred identity ?left-trajectory)
              (lisp-pred identity ?right-trajectory)))

    (-> (lisp-pred identity ?left-grasp-poses)
        (equal ?left-grasp-poses (?look-pose . ?_))
        (equal ?right-grasp-poses (?look-pose . ?_)))

    ;; put together resulting action designator
    (desig:designator :action ((:type :removing-cap)
                               (:object ?current-object-desig)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:effort ?effort)
                               (:left-grasp ?left-grasp)
                               (:right-grasp ?right-grasp)
                               (:location-type ?location-type)
                               (:look-pose ?look-pose)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-grasp-poses ?left-grasp-poses)
                               (:right-grasp-poses ?right-grasp-poses)
                               (:left-remove-cap-poses ?left-openlid-poses)
                               (:right-remove-cap-poses ?right-openlid-poses)
                               (:holding ?holding))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (pick-up2 ?resolved-action-designator))
    (spec:property ?action-designator (:type :holding2))

    ;; extract info from ?action-designator
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))

    ;; get the arm for grasping by checking if it is specified for ?object-type
    (man-int:arms-for-object-type ?object-type ?arms-for-object)
    (-> (equal ?arms-for-object nil)
        (-> (spec:property ?action-designator (:arm ?arm))
            (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
                 (subset ?arm ?free-arms))
            (and (man-int:robot-free-hand ?_ ?free-arm)
                 (equal (?free-arm) ?arm)))
        (-> (spec:property ?action-designator (:arm ?arm))
            (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
                 (subset ?arm ?free-arms)
                 (man-int:check-arms-for-object-type ?arm ?object-type))
            (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
                 (man-int:check-arms-for-object-type ?free-arms ?object-type)
                 (equal ?arm ?free-arms))))
(lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)
    ;; get the type of the picking location, because the trajectory
    ;; might be different depending on the location type
    (once (or (and (spec:property ?current-object-desig (:location ?obj-loc))
                   (desig:current-designator ?obj-loc ?curr-obj-loc)
                   (man-int:location-reference-object ?curr-obj-loc ?obj-loc-obj)
                   (desig:current-designator ?obj-loc-obj ?curr-obj-loc-obj)
                   (spec:property ?curr-obj-loc-obj (:type ?location-type)))
              (equal ?location-type NIL)))

    ;; calculate trajectory with given grasps
    (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)    
    (equal ?objects (?current-object-desig))
    (-> (member :left ?arm)
        (and (-> (spec:property ?action-designator (:left-grasp ?left-grasp))
                 (true)
                 (member ?left-grasp ?grasps))
             (lisp-fun man-int:get-action-trajectory :removing-cap2 :left
                       :top ?location-type ?objects
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grasping
                       ?left-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :lifting
                       ?left-lift-poses))
        (and (equal ?left-grasp NIL)
             (equal ?left-reach-poses NIL)
             (equal ?left-grasp-poses NIL)
             (equal ?left-lift-poses NIL)))

    (-> (member :right ?arm)
        (and  (-> (spec:property ?action-designator (:right-grasp ?right-grasp))
                  (true)
                  (member ?right-grasp ?grasps))
              (lisp-fun man-int:get-action-trajectory :removing-cap2 :right
                        :top ?location-type ?objects
                        ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
                       ?right-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :lifting
                       ?right-lift-poses))
        (and (equal ?right-grasp NIL)
             (equal ?right-reach-poses NIL)
             (equal ?right-grasp-poses NIL)
             (equal ?right-lift-poses NIL)))

    (once (or (lisp-pred identity ?left-trajectory)
              (lisp-pred identity ?right-trajectory)))

    (-> (lisp-pred identity ?left-grasp-poses)
        (equal ?left-grasp-poses (?look-pose . ?_))
        (equal ?right-grasp-poses (?look-pose . ?_)))

    ;; put together resulting action designator
    (desig:designator :action ((:type :holding2)
                               (:object ?current-object-desig)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:effort ?effort)
                               (:left-grasp ?left-grasp)
                               (:right-grasp ?right-grasp)
                               (:location-type ?location-type)
                               (:look-pose ?look-pose)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-grasp-poses ?left-grasp-poses)
                               (:right-grasp-poses ?right-grasp-poses)
                               (:left-lift-poses ?left-lift-poses)
                               (:right-lift-poses ?right-lift-poses)
                               (:hold :holding))
                      ?resolved-action-designator))

  


  (<- (desig:action-grounding ?action-designator (screw-open ?resolved-action-designator))
    (spec:property ?action-designator (:type :screwing-open))
    ;; extract info from ?action-designator
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?_ ?arm))
    
    (lisp-fun man-int:get-object-old-transform ?current-object-desig ?object-transform)

    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (and (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
             (member ?grasp ?grasps)))
    
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

    ;; calculate trajectory
    (equal ?objects (?current-object-desig))
    
    
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :screwing-open ?arm ?grasp T ?objects
                       ?left-screwing-open-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?left-screwing-open-pose :screw-open
                       ?left-screw-open-poses)
        (equal ?left-screw-open-poses NIL)))
    
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory :screwing-open ?arm ?grasp T ?objects
                       ?right-screwing-open-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?right-screwing-open-pose :screw-open
                       ?right-screw-open-poses)
        (equal ?right-slice-up-poses NIL)))

        
    (-> (desig:desig-prop ?action-designator (:collision-mode ?collision-mode))
        (true)
        (equal ?collision-mode nil))
    
    ;;put together resulting action designator
    (desig:designator :action ((:type :screwing-open)
                               (:object ?current-object-desig)
                               (:object-name  ?object-name)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:effort ?effort)
                               (:grasp ?grasp)
                               (:left-screw-open-poses ?left-screw-open-poses)
                               (:right-screw-open-poses ?right-screw-open-poses)
                               (:collision-mode ?collision-mode))
                      ?resolved-action-designator)))


(defun find-and-open-bottle (&key
                    ((:object ?object-designator))
                    ((:context ?context))
                    ((:search-location ?search-location))
                    ((:search-robot-location ?search-base-location))
                    ((:open-bottle-robot-location ?open-bottle-robot-location))
                    ((:arms ?arms))
                    ((:grasps ?grasps))
                  &allow-other-keys)

  ;; ;; if we are not sure about the exact location of deliver-location, find it
  ;; (let ((?goal `(man-int:location-certain ,?delivering-location)))
  ;;   (exe:perform (desig:an action
  ;;                          (type searching)
  ;;                          (location ?delivering-location)
  ;;                          (goal ?goal))))
  ;; ;; if deliver-location is inside a container, open the container
  ;; (let ((?goal `(cpoe:location-accessible ,?delivering-location)))
  ;;   (exe:perform (desig:an action
  ;;                          (type accessing)
  ;;                          (location ?delivering-location)
  ;;                          (goal ?goal))))

  ;; if we are not sure about the exact location of search-location, find it
  (let ((?goal `(man-int:location-certain ,?search-location)))
    (exe:perform (desig:an action
                           (type searching)
                           (location ?search-location)
                           (goal ?goal))))

  ;; if search-location is inside a container, open the container
  (let ((?goal `(cpoe:location-accessible ,?search-location)))
    (exe:perform (desig:an action
                           (type accessing)
                           (location ?search-location)
                           (goal ?goal))))

  ;; search for the object to find it's exact pose
  (exe:perform (desig:an action
                         (type searching)
                         (object ?object-designator)
                         (desig:when ?context
                           (context ?context))
                         (desig:when ?search-base-location
                           (robot-location ?search-base-location))))
  (setf ?object-designator (desig:current-desig ?object-designator))
  (roslisp:ros-info (pp-plans find-and-open-bottle)
                    "Found object of type ~a."
                    (desig:desig-prop-value ?object-designator :type))

  ;; If running on the real robot, execute below task tree
  ;; in projection N times first, then pick the best parameterization
  ;; and use that parameterization in the real world.
  ;; If running in projection,
  ;; just execute the task tree below as normal.
  (let (?open-bottle-pick-up-action ?deliver-place-action)
    (proj-reasoning:with-projected-task-tree
        (?open-bottle-robot-location
         ?open-bottle-pick-up-action
         ?deliver-robot-location
         ?deliver-place-action)
        3
        #'proj-reasoning:pick-best-parameters-by-distance

      ;; fetch the object
      (let ((?open-bottle-goal
              `(cpoe:object-in-hand
                ,?object-designator :left-or-right)))
        (exe:perform (desig:an action
                               (type opening-bottle)
                               (desig:when ?arms
                                 (arms ?arms))
                               ;; (arms (list :right))
                               ;; (desig:when ?grasps
                               ;;   (grasps ?grasps))
                               (grasps (list :right-side))
                               (object ?object-designator)
                               (desig:when ?open-bottle-robot-location
                                 (robot-location ?open-bottle-robot-location))
                               (pick-up-action ?open-bottle-pick-up-action)
                               (goal ?open-bottle-goal))))
      (setf ?object-designator (desig:current-desig ?object-designator))
      (roslisp:ros-info (pp-plans find-and-open-bottle) "Fetched the object.")

      ;; (cpl:with-failure-handling
      ;;     ((common-fail:delivering-failed (e)
      ;;        (declare (ignore e))
      ;;        ;; (return)
      ;;        (drop-at-sink)))

      ;;   ;; deliver at destination
      ;;   (let ((?goal
      ;;           `(cpoe:object-at-location
      ;;             ,?object-designator ,?delivering-location)))
      ;;     (exe:perform (desig:an action
      ;;                            (type delivering)
      ;;                            ;; (desig:when ?arm
      ;;                            ;;   (arm ?arm))
      ;;                            (object ?object-designator)
      ;;                            (context ?context)
      ;;                            (target ?delivering-location)
      ;;                            (desig:when ?deliver-robot-location
      ;;                              (robot-location ?deliver-robot-location))
      ;;                            (place-action ?deliver-place-action)
      ;;                            (goal ?goal)))))
      ))

  ;; reset the fetch location
  (let ((?goal `(cpoe:location-reset ,?search-location)))
    (exe:perform (desig:an action
                           (type sealing)
                           (location ?search-location)
                           (goal ?goal))))


  (desig:current-desig ?object-designator))



(defun open-bottle (&key
                      ((:object ?object-designator))
                      ((:arms ?arms))
                      ((:grasps ?grasps))
                      ((:robot-location ?remove-cap-robot-location))
                      ((:look-location ?look-location))
                      pick-up-action
                      remove-cap-action
                      object-in-hand
                      object-hand
              &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type list ?arms ?grasps)
           ;; ?pick-up-robot-location should not be NULL at the beginning
           ;; but can become NULL during execution of the plan
           (type (or desig:location-designator null) ?remove-cap-robot-location)
           (type (or desig:action-designator null) pick-up-action)
           (type (or desig:action-designator null) remove-cap-action)
           )
  "Fetches a perceived object `?object-designator' with
one of arms in the `?arms' lazy list (if not NIL) and one of grasps in `?grasps' if not NIL,
while standing at `?remove-cap-robot-location'
and using the grasp and arm specified in `remove-cap-action' (if not NIL)."
  (setf ?look-location (desig:reset ?look-location))
  (setf ?remove-cap-robot-location (desig:reset ?remove-cap-robot-location))

  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans fetch) "~a~%Propagating up." e)
         (cpl:fail 'common-fail:fetching-failed
                   :object ?object-designator
                   :description "Some designator could not be resolved.")))

    ;; take a new `?remove-cap-robot-location' sample if a failure happens
    (cpl:with-retry-counters ((relocation-for-ik-retries 50))
      (cpl:with-failure-handling
          (((or common-fail:navigation-goal-in-collision
                common-fail:looking-high-level-failure
                common-fail:perception-low-level-failure
                common-fail:object-unreachable
                common-fail:manipulation-low-level-failure
                desig:designator-error) (e)
             (setf ?remove-cap-robot-location
                   (desig:reset ?remove-cap-robot-location))
             (desig:reference ?remove-cap-robot-location)
             (common-fail:retry-with-loc-designator-solutions
                 ?remove-cap-robot-location
                 relocation-for-ik-retries
                 (:error-object-or-string
                  (format NIL "Object of type ~a is unreachable: ~a"
                          (desig:desig-prop-value ?object-designator :type) e)
                  :warning-namespace (fd-plans fetch)
                  :rethrow-failure 'common-fail:fetching-failed))))

        ;; navigate, look, detect and remove cap
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?remove-cap-robot-location)))

        ;; if fetch location is in hand, we have a handover,
        ;; so move the source hand closer
        (when object-in-hand
          (let ((?goal
                  (case object-hand
                    (:left `(cpoe:arms-positioned-at :hand-over nil))
                    (:right `(cpoe:arms-positioned-at nil :hand-over))
                    (t `(cpoe:arms-positioned-at nil nil)))))
            (exe:perform
             (desig:an action
                       (type positioning-arm)
                       (desig:when (eql object-hand :left)
                         (left-configuration hand-over))
                       (desig:when (eql object-hand :right)
                         (right-configuration hand-over))
                       (goal ?goal)))
            (setf ?look-location (desig:reset ?look-location))))

        (let (;; (?goal `(cpoe:looking-at ,?look-location))
              )
          (exe:perform (desig:an action
                                 (type turning-towards)
                                 (target ?look-location)
                                 ;; (goal ?goal)
                                 )))

        (cpl:with-retry-counters ((regrasping-retries 1))
          (cpl:with-failure-handling
              ((common-fail:gripper-low-level-failure (e)
                 (roslisp:ros-warn (fd-plans fetch) "Misgrasp happened: ~a~%" e)
                 (exe:perform (desig:an action
                                        (type releasing)
                                        (gripper left)))
                 (exe:perform (desig:an action
                                        (type releasing)
                                        (gripper right)))
                 (cpl:do-retry regrasping-retries
                   (roslisp:ros-info (fd-plans fetch) "Reperceiving and repicking...")
                   (cpl:retry))
                 (roslisp:ros-warn (fd-plans fetch) "No more regrasping retries left :'(")
                 (cpl:fail 'common-fail:object-unreachable
                           :description "Misgrasp happened and retrying didn't help.")))

            (let ((?more-precise-perceived-object-desig
                    (exe:perform (desig:an action
                                           (type perceiving)
                                           (object ?object-designator)))))

              (let ((?arm (list (cut:lazy-car ?arms))))
                ;; if picking up fails, try another arm
                (cpl:with-retry-counters ((arm-retries 1))
                  (cpl:with-failure-handling
                      (((or common-fail:manipulation-low-level-failure
                            common-fail:object-unreachable
                            desig:designator-error) (e)
                         (common-fail:retry-with-list-solutions
                             ?arms
                             arm-retries
                             (:error-object-or-string
                              (format NIL "Manipulation failed: ~a.~%Next." e)
                              :warning-namespace (fd-plans fetch)
                              :rethrow-failure 'common-fail:object-unreachable)
                           (setf ?arm (list (cut:lazy-car ?arms))))))

                    (print ?arm)
                    (print ?arms)
                    ;; (break)
                    (let ((?grasp (cut:lazy-car ?grasps)))
                      ;; if picking up fails, try another grasp orientation
                      (cpl:with-retry-counters ((grasp-retries 4))
                        (cpl:with-failure-handling
                            (((or common-fail:manipulation-low-level-failure
                                  common-fail:object-unreachable
                                  desig:designator-error) (e)
                               (common-fail:retry-with-list-solutions
                                   ?grasps
                                   grasp-retries
                                   (:error-object-or-string
                                    (format NIL "Picking up failed: ~a.~%Next" e)
                                    :warning-namespace (fd-plans fetch))
                                 (setf ?grasp (cut:lazy-car ?grasps)))))
                          ;; (exe:perform
                          ;;  (desig:an action
                          ;;            (type holding)
                          ;;            (arm :left)
                          ;;  (desig:when ?grasp
                          ;;    (grasp ?grasp))
                          ;;  (object
                          ;;   ?more-precise-perceived-object-desig)
                          ;;  ;; (goal ?goal)
                          ;;  ))
                          (let* ((?goal
                                   `(cpoe:object-in-hand
                                     ,?more-precise-perceived-object-desig
                                     :left-or-right))
                                 (?other-arm (cond
                                               ((member :left ?arm) (list :right))
                                               (T (list :left))))
                                 (pick-up-action
                                 ;; (remove-cap-action
                                  ;; if remove-cap-action already exists,
                                  ;; use its params for removing cap
                                  (or (when pick-up-action
                                        (let* ((referenced-action-desig
                                                 (desig:reference pick-up-action))
                                               (?arm
                                                 (list (desig:desig-prop-value
                                                        referenced-action-desig
                                                        :arm)))
                                               (?grasp
                                                 (desig:desig-prop-value
                                                  referenced-action-desig
                                                  :grasp)))
                                          (desig:an action
                                                    (type holding)
                                                    (arm ?arm)
                                                    (grasp :front)
                                                    (object
                                                     ?more-precise-perceived-object-desig)
                                                    ;; (goal ?goal)
                                                    )))
                                      (desig:an action
                                                (type holding)
                                                (desig:when ?arm
                                                  (arm ?arm))
                                                (desig:when ?grasp
                                                  (grasp :front))
                                                (object
                                                 ?more-precise-perceived-object-desig)
                                                ;; (goal ?goal)
                                                )))
                                 
                                 (remove-cap-action
                                  ;; if remove-cap-action already exists,
                                  ;; use its params for removing cap
                                  (or (when remove-cap-action
                                        (let* ((referenced-action-desig
                                                 (desig:reference remove-cap-action))
                                               (?arm
                                                 (list (desig:desig-prop-value
                                                        referenced-action-desig
                                                        :arm)))
                                               (?grasp
                                                 (desig:desig-prop-value
                                                  referenced-action-desig
                                                  :grasp)))
                                          (desig:an action
                                                    (type holding2)
                                                    (arm ?other-arm)
                                                    (grasp :top)
                                                    (object
                                                     ?more-precise-perceived-object-desig)
                                                    ;; (goal ?goal)
                                                    )))
                                      (desig:an action
                                                (type holding2)
                                                (desig:when ?arm
                                                  (arm ?other-arm))
                                                (desig:when ?grasp
                                                  (grasp :top))
                                                (object
                                                 ?more-precise-perceived-object-desig)
                                                ;; (goal ?goal)
                                                )))
                                 )
                            (print ">>>>>>>>>>>>>>BREAK 0")
                            (print ?other-arm)
                            (print ?arm)
                            ;; (break)
                            (setf pick-up-action (desig:current-desig pick-up-action))
                            (proj-reasoning:check-picking-up-collisions pick-up-action)
                            (setf pick-up-action (desig:current-desig pick-up-action))
                            (exe:perform pick-up-action)
                            ;; (print ">>>>>>>>>>>>>>BREAK 1")
                            (break)
                            (setf remove-cap-action (desig:current-desig remove-cap-action))
                            (proj-reasoning::check-picking-up-collisions remove-cap-action) 
                            (setf remove-cap-action (desig:current-desig remove-cap-action))                            
                            (exe:perform remove-cap-action)
                            (print ">>>>>>>>>>>>>>BREAK 2")
                            ;;(break)
                            
                            (desig:current-desig ?object-designator)))))))))))))))


(defun remove-cap (&key
                  ((:object ?object-designator))
                  ((:arm ?arm))
                  ((:gripper-opening ?gripper-opening))
                  ((:effort ?grip-effort))
                  ((:left-grasp ?left-grasp))
                  ((:right-grasp ?right-grasp))
                  location-type
                  ((:look-pose ?look-pose))
                  ((:left-reach-poses ?left-reach-poses))
                  ((:right-reach-poses ?right-reach-poses))
                  ((:left-grasp-poses ?left-grasp-poses))
                  ((:right-grasp-poses ?right-grasp-poses))
                  ((:left-remove-cap-poses ?left-remove-cap-poses))
                  ((:right-remove-cap-poses ?right-remove-cap-poses))
                  ((:hold ?holding))
                &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type list ?arm)
           ;;(type (or nil keyword) ?left-grasp ?right-grasp)
           (type number ?gripper-opening ?grip-effort)
           (type (or null list) ; yes, null is also a list, but this is more readable
                 ?left-reach-poses ?right-reach-poses
                 ?left-grasp-poses ?right-grasp-poses
                 ?left-remove-cap-poses ?right-remove-cap-poses)
           (ignore location-type))
  "Open gripper, reach traj, grasp traj, close gripper, issue grasping event, lift."

  (cram-tf:visualize-marker (man-int:get-object-pose ?object-designator)
                            :r-g-b-list '(1 1 0) :id 300)

  (roslisp:ros-info (pick-place pick-up) "Looking")
  (cpl:with-failure-handling
      ((common-fail:ptu-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Looking-at had a problem: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type looking)
               (target (desig:a location
                                (pose ?look-pose))))))
  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper and reaching")
    (let ((?goal `(cpoe:gripper-joint-at ,?arm ,?gripper-opening)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening)
                 (goal ?goal))))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
        (exe:perform
         (desig:an action
                   (type reaching)
                   (object ?object-designator)
                   (left-poses ?left-reach-poses)
                   (right-poses ?right-reach-poses)
                   (goal ?goal))))))
  (roslisp:ros-info (pick-place pick-up) "Grasping")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(cpoe:tool-frames-at ,?left-grasp-poses ,?right-grasp-poses)))
      (exe:perform
       (desig:an action
                 (type grasping)
                 (object ?object-designator)
                 (left-poses ?left-grasp-poses)
                 (right-poses ?right-grasp-poses)
                 (goal ?goal)))))
  ;; (let ((?goal `(cpoe:object-in-hand ,?object-designator ,?arm)))
  ;;   (roslisp:ros-info (pick-place pick-up) "Gripping")
  ;;   (when (member :left ?arm)
  ;;     (exe:perform
  ;;      (desig:an action
  ;;                (type gripping)
  ;;                (gripper :left)
  ;;                (effort ?grip-effort)
  ;;                (object ?object-designator)
  ;;                (desig:when ?left-grasp
  ;;                  (grasp ?left-grasp))
  ;;                (goal ?goal))))
  ;;   (when (member :right ?arm)
  ;;     (exe:perform
  ;;      (desig:an action
  ;;                (type gripping)
  ;;                (gripper :right)
  ;;                (effort ?grip-effort)
  ;;                (object ?object-designator)
  ;;                (desig:when ?right-grasp
  ;;                  (grasp ?right-grasp))
  ;;                (goal ?goal)))))
  (unless ?holding
   (roslisp:ros-info (pick-place pick-up) "Lifting")
   (cpl:pursue
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (let ((?goal `(cpoe:tool-frames-at ,?left-remove-cap-poses ,?right-remove-cap-poses)))
        (exe:perform
         (desig:an action
                   (type screw-open)
                   (left-poses ?left-remove-cap-poses)
                   (right-poses ?right-remove-cap-poses)
                   (goal ?goal)))))
    ;; (cpl:seq
    ;;   (exe:perform
    ;;    (desig:an action
    ;;              (type monitoring-joint-state)
    ;;              (gripper ?arm)))
    ;;   (cpl:fail 'common-fail:gripper-closed-completely
    ;;             :description "Object slipped"))
     )
  (roslisp:ros-info (pick-place place) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             ;; TODO: this will not work with dual-arm grasping
             ;; but as our ?arm is declared as a keyword,
             ;; for now this code is the right code
             (arms (?arm))))))

(defun pick-up2 (&key
                  ((:object ?object-designator))
                  ((:arm ?arm))
                  ((:gripper-opening ?gripper-opening))
                  ((:effort ?grip-effort))
                  ((:left-grasp ?left-grasp))
                  ((:right-grasp ?right-grasp))
                  location-type
                  ((:look-pose ?look-pose))
                  ((:left-reach-poses ?left-reach-poses))
                  ((:right-reach-poses ?right-reach-poses))
                  ((:left-grasp-poses ?left-grasp-poses))
                  ((:right-grasp-poses ?right-grasp-poses))
                  ((:left-lift-poses ?left-lift-poses))
                  ((:right-lift-poses ?right-lift-poses))
                  ((:hold ?holding))
                &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type list ?arm)
           ;;(type (or nil keyword) ?left-grasp ?right-grasp)
           (type number ?gripper-opening ?grip-effort)
           (type (or null list) ; yes, null is also a list, but this is more readable
                 ?left-reach-poses ?right-reach-poses
                 ?left-grasp-poses ?right-grasp-poses
                 ?left-lift-poses ?right-lift-poses)
           (ignore location-type))
  "Open gripper, reach traj, grasp traj, close gripper, issue grasping event, lift."
  (cram-tf:visualize-marker (man-int:get-object-pose ?object-designator)
                            :r-g-b-list '(1 1 0) :id 300)
  (print "print arm")
  (print ?arm)
  (print "?left-reach-poses")
  (print ?left-reach-poses)
  (print "?right-reach-poses")
  (print ?right-reach-poses)
  (print "?left-grasp-poses")
  (print ?left-grasp-poses)
  (print "?right-grasp-poses")
  (print ?right-grasp-poses)
  (print "?left-lift-poses")
  (print ?left-lift-poses)
  (print "?right-lift-poses")
  (print ?right-lift-poses)

  ;; (break)
  (roslisp:ros-info (pick-place pick-up) "Looking")
  (cpl:with-failure-handling
      ((common-fail:ptu-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Looking-at had a problem: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type looking)
               (target (desig:a location
                                (pose ?look-pose))))))
  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper and reaching")
    (let ((?goal `(cpoe:gripper-joint-at ,?arm ,?gripper-opening)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening)
                 (goal ?goal))))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
        (exe:perform
         (desig:an action
                   (type reaching)
                   (object ?object-designator)
                   (left-poses ?left-reach-poses)
                   (right-poses ?right-reach-poses)
                   (goal ?goal))))))
  (roslisp:ros-info (pick-place pick-up) "Grasping")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(cpoe:tool-frames-at ,?left-grasp-poses ,?right-grasp-poses)))
      (exe:perform
       (desig:an action
                 (type grasping)
                 (object ?object-designator)
                 (left-poses ?left-grasp-poses)
                 (right-poses ?right-grasp-poses)
                 (goal ?goal)))))
  ;; (let ((?goal `(cpoe:object-in-hand ,?object-designator ,?arm)))
  ;;   (roslisp:ros-info (pick-place pick-up) "Gripping")
  ;;   (when (member :left ?arm)
  ;;     (exe:perform
  ;;      (desig:an action
  ;;                (type gripping)
  ;;                (gripper :left)
  ;;                (effort ?grip-effort)
  ;;                (object ?object-designator)
  ;;                (desig:when ?left-grasp
  ;;                  (grasp ?left-grasp))
  ;;                (goal ?goal))))
  ;;   (when (member :right ?arm)
  ;;     (exe:perform
  ;;      (desig:an action
  ;;                (type gripping)
  ;;                (gripper :right)
  ;;                (effort ?grip-effort)
  ;;                (object ?object-designator)
  ;;                (desig:when ?right-grasp
  ;;                  (grasp ?right-grasp))
  ;;                (goal ?goal)))))
  (unless ?holding
   (roslisp:ros-info (pick-place pick-up) "Lifting")
   (cpl:pursue
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (let ((?goal `(cpoe:tool-frames-at ,?left-lift-poses ,?right-lift-poses)))
        (exe:perform
         (desig:an action
                   (type lifting)
                   (left-poses ?left-lift-poses)
                   (right-poses ?right-lift-poses)
                   (goal ?goal)))))
    ;; (cpl:seq
    ;;   (exe:perform
    ;;    (desig:an action
    ;;              (type monitoring-joint-state)
    ;;              (gripper ?arm)))
    ;;   (cpl:fail 'common-fail:gripper-closed-completely
    ;;             :description "Object slipped"))
     )
  (roslisp:ros-info (pick-place place) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             ;; TODO: this will not work with dual-arm grasping
             ;; but as our ?arm is declared as a keyword,
             ;; for now this code is the right code
             (arms (?arm))))))


(defun screw-open (&key
                ((:object ?object-designator))
                ((:object-name  ?object-name))
                ((:arm ?arm))
                ((:arm-support ?arm-support))
                ((:gripper-opening ?gripper-opening))
                ((:effort ?grip-effort))
                ((:grasp ?grasp))
                ((:grasp-support ?grasp-support))
                ((:left-screw-open-poses ?left-screw-open-poses))
                ((:right-screw-open-poses ?right-screw-open-poses))
                ((:collision-mode ?collision-mode))
              &allow-other-keys)
  "Object is secured by other hand (desig holding)"
  

  (loop while (or ?left-screw-open-poses
                  ?right-screw-open-poses)
        do
           
           (let ((?current-left-screw-open-poses `(,(pop ?left-screw-open-poses)))
                 (?current-right-screw-open-poses `(,(pop ?right-screw-open-poses))))

             (roslisp:ros-info (cut-pour pour) "approach")
             (cpl:with-failure-handling
                 ((common-fail:manipulation-low-level-failure (e)
                    (roslisp:ros-warn (cut-and-pour-plans slice)
                                      "Manipulation messed up: ~a~%Ignoring."
                                      e)))
               (exe:perform
                (desig:an action
                          (type approaching)
                          (left-poses ?current-left-screw-open-poses)
                          (right-poses ?current-right-screw-open-poses)
                          (desig:when ?collision-mode
                            (collision-mode ?collision-mode))))))))
