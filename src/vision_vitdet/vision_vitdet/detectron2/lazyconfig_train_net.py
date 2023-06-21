#!/usr/bin/env python3
# Copyright (c) Facebook, Inc. and its affiliates.
"""
Training script using the new "LazyConfig" python config files.

This scripts reads a given python config file and runs the training or evaluation.
It can be used to train any models or dataset as long as they can be
instantiated by the recursive construction defined in the given config file.

Besides lazy construction of models, dataloader, etc., this scripts expects a
few common configuration parameters currently defined in "configs/common/train.py".
To add more complicated training logic, you can easily add other configs
in the config file and implement a new train_net.py to handle them.
"""
import logging

import detectron2

from detectron2 import model_zoo

from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import LazyConfig, instantiate
from detectron2.engine import (
    AMPTrainer,
    SimpleTrainer,
    default_argument_parser,
    default_setup,
    default_writers,
    hooks,
    launch,
)
from detectron2.engine.defaults import create_ddp_model
from detectron2.evaluation import inference_on_dataset, print_csv_format
from detectron2.utils import comm
from detectron2.data.datasets import register_coco_instances
from detectron2.data.samplers import RepeatFactorTrainingSampler
import sys
sys.path.insert(0, './projects/ViTDet/configs/common')
from projects.ViTDet.configs.common.coco_loader_lsj import dataloader
from detectron2.config import LazyCall as L
from detectron2.data import MetadataCatalog
from detectron2.data.catalog import DatasetCatalog
from detectron2.data import (
    DatasetMapper,
    build_detection_test_loader,
    build_detection_train_loader,
    get_detection_dataset_dicts,
)
from omegaconf import DictConfig, ListConfig, OmegaConf, SCMode
from detectron2.evaluation import COCOEvaluator
from projects.ViTDet.configs.COCO.mask_rcnn_vitdet_b_100ep import (
    dataloader,
    lr_multiplier,
    model,
    train,
    optimizer,
    get_vit_lr_decay_rate,
)

logger = logging.getLogger("detectron2")

config_file = "./projects/ViTDet/configs/COCO/mask_rcnn_vitdet_b_100ep.py"

def do_test(cfg, model):
    model.eval()
    print("AAAAAAAAAAAAAAAAAAA")
    #cfg.model.backbone.net.img_size = 1024
    if "evaluator" in cfg.dataloader:
        ret = inference_on_dataset(
            model, instantiate(cfg.dataloader.test), instantiate(cfg.dataloader.evaluator)
        )
        print_csv_format(ret)
        return ret


def do_train(args, cfg):
    """
    Args:
        cfg: an object with the following attributes:
            model: instantiate to a module
            dataloader.{train,test}: instantiate to dataloaders
            dataloader.evaluator: instantiate to evaluator for test set
            optimizer: instantaite to an optimizer
            lr_multiplier: instantiate to a fvcore scheduler
            train: other misc config defined in `configs/common/train.py`, including:
                output_dir (str)
                init_checkpoint (str)
                amp.enabled (bool)
                max_iter (int)
                eval_period, log_period (int)
                device (str)
                checkpointer (dict)
                ddp (dict)
    """
    model = instantiate(cfg.model)
    logger = logging.getLogger("detectron2")
    logger.info("Model:\n{}".format(model))
    model.to(cfg.train.device)

    cfg.optimizer.params.model = model
    optim = instantiate(cfg.optimizer)

    train_loader = instantiate(cfg.dataloader.train)

    model = create_ddp_model(model, **cfg.train.ddp)
    trainer = (AMPTrainer if cfg.train.amp.enabled else SimpleTrainer)(model, train_loader, optim)
    checkpointer = DetectionCheckpointer(
        model,
        cfg.train.output_dir,
        trainer=trainer,
    )
    trainer.register_hooks(
        [
            hooks.IterationTimer(),
            hooks.LRScheduler(scheduler=instantiate(cfg.lr_multiplier)),
            hooks.PeriodicCheckpointer(checkpointer, **cfg.train.checkpointer)
            if comm.is_main_process()
            else None,
            hooks.EvalHook(cfg.train.eval_period, lambda: do_test(cfg, model)),
            hooks.PeriodicWriter(
                default_writers(cfg.train.output_dir, cfg.train.max_iter),
                period=cfg.train.log_period,
            )
            if comm.is_main_process()
            else None,
        ]
    )

    checkpointer.resume_or_load(cfg.train.init_checkpoint, resume=args.resume)
    if args.resume and checkpointer.has_checkpoint():
        # The checkpoint stores the training iteration that just finished, thus we start
        # at the next iteration
        start_iter = trainer.iter + 1
    else:
        start_iter = 0
    trainer.train(start_iter, cfg.train.max_iter)



def main(args):
    cfg = LazyConfig.load(config_file)
    
    default_setup(cfg, args)

    register_coco_instances("ball_train", {},"./train/_annotations.coco.json", "./train")
    register_coco_instances("ball_test", {}, "./test/_annotations.coco.json", "./test")

    cfg.dataloader.train.dataset.names = "ball_train"
    cfg.dataloader.test.dataset.names = "ball_test"
    dataset_dicts = DatasetCatalog.get("ball_train")
    print("oooooooooooooooooooooooooooooooooooooooooooooooooo")
    print(MetadataCatalog.get("ball_train"))
    MetadataCatalog.get("ball_train").set(thing_classes=["ball"])
    MetadataCatalog.get("ball_train").thing_classes=["ball"]
    MetadataCatalog.get("ball_test").set(thing_classes=["ball"])
    MetadataCatalog.get("ball_test").thing_classes=["ball"]
    metadata = MetadataCatalog.get(cfg.dataloader.train.dataset.names) # to get labels from ids
    metadata_test = MetadataCatalog.get(cfg.dataloader.test.dataset.names) # to get labels from ids
    cfg.dataloader.evaluator.output_dir = 'ball_train_1000'
    #cfg.dataloader.evaluator = COCOEvaluator("ball_test",tasks={"bbox"},distributed = False, output_dir="ball_train")
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" + str(OmegaConf.to_yaml(cfg)))
    
   
    if args.eval_only:
        train.init_checkpoint="./train_ball_1000/model_final.pth"
        print(train)
        cfg.model.roi_heads.num_classes = 1
        cfg.model.backbone.norm = "BN"
        model = instantiate(cfg.model)
        model.to(cfg.train.device)
        model = create_ddp_model(model)
        DetectionCheckpointer(model).load(train.init_checkpoint)
        print(do_test(cfg, model))
    else:
        cfg.model.backbone.norm = "BN"
        print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
        print(cfg.dataloader.train)
        print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
        cfg.train.max_iter=1000
        cfg.train.output_dir='./train_ball_1000'
        cfg.model.roi_heads.num_classes = 1
        cfg.optimizer.lr=0.0005
        cfg.dataloader.train.total_batch_size = 2
        cfg.train.device = "cuda"
        
        # cfg.INPUT.MIN_SIZE_TRAIN = 20
        cfg = LazyConfig.apply_overrides(cfg, args.opts)
        do_train(args, cfg)


if __name__ == "__main__":
    args = default_argument_parser().parse_args()
    launch(
        main,
        args.num_gpus,
        num_machines=args.num_machines,
        machine_rank=args.machine_rank,
        dist_url=args.dist_url,
        args=(args,),
    )

