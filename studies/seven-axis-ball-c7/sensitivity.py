from study import batch,SELECTED
if __name__=="__main__":
    # Same continuous perturbation, controller/measurement interval and noise.
    batch([(c,m,.000125) for c in ("push_y3","val_push_down","down_38","return_side")
           for m in ("off","both",SELECTED)],4)
