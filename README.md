# opensim-moco-issue

# Run
`python testMocoTrack.py`

##  .trc files
These files were collected using a mocap system at 240Hz

## model.osim
The model has been scaled following OpenSim tutorial

## Errors
- "motion_1_modified.trc"
    - nlp:nlp_g failed: NaN detected for output g, at (row 1873, col 0).
- "motion_2_modified.trc"
    - nlp:nlp_grad_f failed: NaN detected for output grad_f_x, at (row 0, col 0).
    - nlp:nlp_jac_g failed: NaN detected for output jac_g_x, at nonzero index 5947 (row 7147, col 0).
    - Which eventually leads to, "Error evaluating objective gradient at user provided starting point. No scaling factor for objective function computed!"
- "motion_3_modified.trc"
    - Similar to motion_1
