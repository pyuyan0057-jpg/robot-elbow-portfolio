from experiment import ProcessPoolExecutor,run
if __name__=="__main__":
    jobs=[("ball",case,mode) for mode in ("thumb_bias","index_bias","small") for case in
          ("nominal","val_push_down","val_push_diag","push_nx3","push_y6")]
    with ProcessPoolExecutor(7) as pool:
        list(pool.map(run,jobs))
