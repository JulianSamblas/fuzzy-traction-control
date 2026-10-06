; =====================================================================
; KNOWLEDGE BASE - Predictive traction control for racing motorcycles
; Approximate Reasoning - Julian Samblas Caballero
; FuzzyCLIPS 6.10d
; (variable and label names kept in Spanish, as in the original report)
; =====================================================================

; ------------------- LINGUISTIC VARIABLES ----------------------------
(deftemplate deslizamiento          ; rear-wheel slip (%)
   0 20
   (  (bajo  (0 1) (4 1) (10 0))
      (medio (4 0) (10 1) (16 0))
      (alto  (10 0) (16 1) (20 1))  )
)

(deftemplate inclinacion            ; lean angle (degrees)
   0 60
   (  (baja  (0 1) (15 1) (30 0))
      (media (15 0) (30 1) (45 0))
      (alta  (30 0) (45 1) (60 1))  )
)

(deftemplate recorte                ; torque cut (%)
   0 100
   (  (nulo   (0 1) (10 1) (30 0))
      (suave  (10 0) (30 1) (50 0))
      (medio  (30 0) (50 1) (70 0))
      (fuerte (50 0) (70 1) (100 1))  )
)

; ----------------------- RULE BASE -----------------------------------
; R1: high slip with the bike leaned over -> highside risk
(defrule regla_1
   (deslizamiento alto)
   (inclinacion alta)
=>
   (assert (recorte fuerte))
)

; R2: high slip with the bike upright -> controllable drift
(defrule regla_2
   (deslizamiento alto)
   (inclinacion baja)
=>
   (assert (recorte medio))
)

; R3: moderate slip with the bike leaned over -> little contact patch
(defrule regla_3
   (deslizamiento medio)
   (inclinacion alta)
=>
   (assert (recorte medio))
)

; R4: moderate slip at medium lean -> fine correction
(defrule regla_4
   (deslizamiento medio)
   (inclinacion media)
=>
   (assert (recorte suave))
)

; R5: no slip -> do not intervene (lean angle not considered)
(defrule regla_5
   (deslizamiento bajo)
=>
   (assert (recorte nulo))
)
