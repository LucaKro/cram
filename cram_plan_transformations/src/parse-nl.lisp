;;; todo license

(in-package :cram-plan-transformations)

(defun transform-into-plist (a-tree &optional (a-plist nil))
  "Transforms a tree into a property list. `a-tree' is something like
   `((action-role flip-1      action-verb)
     (has-sense   flip-1      flip-1-8)
     (is-a        flip-1-8    flip.v.08)
     (action-role pancake-3   theme)
     (has-sense   pancake-3   pancake-3-1)
     (is-a        pancake-3-1 pancake.n.01))
   for which the result would be
   `((THEME PANCAKE.N.01) (ACTION-VERB FLIP.V.08))"
  (let ((tripple (car a-tree)))
   (if (null a-tree)
       a-plist
       (cond ((member 'action-role tripple)
              (transform-into-plist
               (cdr a-tree)
               (cons (list (third tripple) (second tripple)) a-plist)))
             ((or (member 'has-sense tripple)
                  (member 'is-a tripple))
              (transform-into-plist
               (cdr a-tree)
               (mapcar #'(lambda (x) ; maybe it could stop at the first occurrence
                         (if (eql (car (cdr x)) (second tripple))
                             (list (car x) (third tripple))
                             x)) a-plist)))))))

(defun get-owl-name (wordnet-name)
  (gensym "owl-bla"))

(defun extract-the-actual-name (name-with-id-stuff)
  "e.g. flip.v.08 -> flip"
  (let* ((a-string (symbol-name name-with-id-stuff))
         (index (position #\. a-string :from-end t :end (position #\. a-string :from-end t)))
         (a-substring (subseq a-string 0 index)))
    (intern a-substring)))

(def-fact-group cram-plan-transformations ()
  ;; TODO comments
  (<- (transform-plist-into-descriptions ?plist ?descriptions)
    (findall ?description
             (and (member ?key-value-pair ?plist)
                  (description-for-key-value-pair ?key-value-pair ?description))
             ?descriptions))

  (<- (description-for-key-value-pair (action-verb ?wordnet-name) ?description)
    (lisp-fun extract-the-actual-name ?wordnet-name ?name)
    (lisp-fun get-owl-name ?wordnet-name ?owl-name)
    (equal ?description (instruction (a action (name ?owl-name) (type ?name)))))

  (<- (description-for-key-value-pair (theme ?wordnet-name) ?description)
    (lisp-fun extract-the-actual-name ?wordnet-name ?name)
    (lisp-fun get-owl-name ?wordnet-name ?owl-name)
    (equal ?description (theme (a object (name ?owl-name) (type ?name)))))

  (<- (description-for-key-value-pair (instrument ?wordnet-name) ?description)
    (lisp-fun extract-the-actual-name ?wordnet-name ?name)
    (lisp-fun get-owl-name ?wordnet-name ?owl-name)
    (equal ?description (instrument (a object (name ?owl-name) (type ?name)))))

  (<- (transform-descriptions-into-action-description ?descriptions ?action-description)
    (member ?description ?descriptions)
    (format "members:~a~%" ?description)
    (equal ?description (instruction ?stuff))
    (findall ?action-property
             (and (member ?action-property ?stuff)
                  (not (equal ?action-property a))
                  (not (equal ?action-property action)))
             ?action-properties)
    (format "action-properties: ~a~%" ?action-properties)
    (findall ?remaining-description
             (and (member ?remaining-description ?descriptions)
                  (not (equal ?remaining-description ?description)))
             ?remaining-descriptions)
    (equal ?action-description (instruction (a action
                                              ?action-properties
                                              ?remaining-descriptions))))

  (<- (transform-plist-into-action-description ?plist ?action-description)
    (transform-plist-into-descriptions ?plist ?descriptions)
    (transform-descriptions-into-action-description ?descriptions ?action-description))

  (<- (add-items-to-action-description ?description ?items-plist ?new-description)
    ;; first transform the `?items-plist' into descriptions
    (transform-plist-into-descriptions ?items-plist ?items-description)
    ;; then append the descriptions to the action description
    (equal ?description (instruction (a action ?key-value-pairs)))
    (append ?key-value-pairs ?items-description ?augmented-key-value-pairs)
    (equal ?new-description (instruction (a action ?augmented-key-value-pairs)))))

;;; STEP 4: compare goal description to general plan and do something ...:D
;;; either use create-designator or compare the two and transform the plan as a list

;; (INSTRUCTION (A ACTION
;;                (TYPE FLIP)
;;                (NAME OWLBLABLA)
;;                (THEME (A OBJECT
;;                         (NAME #:|owl-bla1268|)
;;                         (TYPE PANCAKE)))
;;                (INSTRUMENT (A OBJECT
;;                              (NAME #:|owl-bla122369|)
;;                              (TYPE SPATULA)))
;;                (GOAL (-> (holds (and (at theme ?location)
;;                                      (orientation theme ?ori))
;;                                 (task-start OWLBLABLA))
;;                          (holds (and (at theme ?location)
;;                                      (orientation theme 90flipped))
;;                                 (task-end OWLBLABLA))
            
;;              )) ;; goal comes from action plan 
;;    (FLAWS (pancake-lost-shape))))

;; (achieve flip pancake-orientation-90degrees)
;; as in (achieve action with-goal)


;; ?instr = "flip the pancake" | (frame-representation  ;  <- call
;; --------- v --------------  |   ?instr               ;      to
;; (a action                   |   ?slot-value-pairs    ;     Daniel
;;   (name owlrelatedname)     |                        ;
;;   (type ?action-core)       |                        ; ?slot-value-pairs:
;;   ?slot-value-pairs)        |                        ;  ((verb flip)
;;                             |                        ;   (theme pancake))

;; (a action                   | (needed-roles          ; <- cram plans
;;   (name owlrelatedname)     |   ?action-core         ;
;;   (type ?action-core)       |   ?needed-roles)       ;
;;   ?slot-value-pairs)        | (given-roles           ;
;; ----------- v ------------- |   ?slot-value-pairs)   ; <- prolog
;; (with-designators           | (missing-roles         ;
;;     ...                     |   ?needed-roles        ; <- gaya
;;   (achieve ...))            |   ?slot-value-pairs    ;
;;                             |   ?missing-roles)      ;
