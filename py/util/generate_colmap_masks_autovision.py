import argparse
from pathlib import Path
from shutil import copy

def copy_masks(args):
    for segment_ind in range(1, args.num_segments+1):
        segment_dir = args.autovision_root / f'{segment_ind}'
        args.output_masks_dir.mkdir(parents=True, exist_ok=True)

        device_names = [str(p.name) for p in segment_dir.iterdir() if p.is_dir()]
        device_names.sort()
        assert len(device_names) != 0
        image_names = [str(p.name) for p in (segment_dir / device_names[0]).iterdir()]
        image_names.sort()
        
        for dev in device_names:
            dev_masks_dir = args.output_masks_dir / f'{segment_ind}' / dev
            dev_masks_dir.mkdir(parents=True, exist_ok=True)
            mask_path = args.autovision_root / f'calib/mask_{dev}.png'
            for img in image_names:
                copy(mask_path, dev_masks_dir / f'{img}.png')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('autovision_root', type=Path)
    parser.add_argument('output_masks_dir', type=Path, default=Path('pairs.txt'))
    parser.add_argument('--num_segments', type=int, default=5)
    args = parser.parse_args()
    
    copy_masks(args)
