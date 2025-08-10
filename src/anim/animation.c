/*
    nt5 -- Windows XP simulator.
    Copyright (C) 2024  Sergei Baigerov

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Contact Sergei Baigerov -- @dogotrigger in Discord
*/

#define DEBUG 0

#include "renderer_keyframe.h"
#include "renderer_animation.h"
#include "renderer_ease.h"
#include <stddef.h>
#include <stdlib.h>

void _ntRendererResetAnimationB(struct renderer_animation* animation) {
    if (!animation || !animation->valid) return;

    animation->completed = 0;
    animation->completed_local = 0;
    animation->current_keyframe = 0;
    animation->current_value = animation->final_value;
    animation->starting_value = animation->current_value;
    animation->time = 0;
    animation->itime = 0;

    if (animation->linked_animation != NULL) {
        _ntRendererResetAnimationB(animation->linked_animation);
    }
}

void _ntRendererUpdateAnimation(struct renderer_animation *animation) {
    if (!animation) {
#if DEBUG == 1
        TraceLog(LOG_INFO, "Animation is NULL");
#endif
        return;
    }

    animation->valid = 0;

    if (animation->count == 0) {
#if DEBUG == 1
        TraceLog(LOG_INFO, "[%d] animation do not have keyframes", animation->anim_id);
#endif
        return;
    }
    if (animation->keyframes == NULL) {
#if DEBUG == 1
        TraceLog(LOG_INFO, "[%d] keyframes are invalid", animation->anim_id);
#endif
        return;
    }

    animation->valid = 1;

#if DEBUG == 1
    // TraceLog(LOG_INFO, "[%d] current keyframe: %d; time: %f", animation->anim_id, animation->current_keyframe, (float)animation->time);
#endif

    if (animation->completed && animation->looping) {
#if DEBUG == 1
        TraceLog(LOG_INFO, "[%d] repeating whole animation", animation->anim_id);
#endif

        if (animation->reset_after_loop) {
            _ntRendererResetAnimation(animation);
        } else {
            _ntRendererResetAnimationB(animation);
        }
    }

    if (animation->linked_animation != NULL) {
        struct renderer_animation *anim = (struct renderer_animation *)animation->linked_animation;
        anim->delta = animation->delta;

        _ntRendererUpdateAnimation(anim);

        animation->completed = animation->completed_local && anim->completed_local;
    } else {
        animation->completed = animation->completed_local;
    }

    struct renderer_keyframe *selected = animation->keyframes + animation->current_keyframe;

    if (animation->current_keyframe < animation->count) {
        if (animation->itime >= selected->length) {
            animation->itime = 0;
            animation->time += selected->length;
            animation->current_keyframe++;
            animation->final_value = animation->starting_value + selected->ending_value;

#if DEBUG == 1
            TraceLog(LOG_INFO, "[%d] keyframe %d completed", animation->anim_id, animation->current_keyframe);
#endif

            if (animation->current_keyframe >= animation->count) {
                animation->completed_local = 1;
#if DEBUG == 1
                TraceLog(LOG_INFO, "[%d] animation completed", animation->anim_id);
#endif
                return;
            }

            animation->starting_value += selected->ending_value;

            selected = animation->keyframes + animation->current_keyframe;
        }
    }

    static double (*easings[TOEnd])(double) = {
        _rendererLinear, _rendererInSine, _rendererOutSine,
        _rendererInOutSine, _rendererInQuad, _rendererOutQuad,
        _rendererInOutQuad, _rendererInCubic, _rendererOutCubic,
        _rendererInOutCubic, _rendererInQuart, _rendererOutQuart,
        _rendererInOutQuart, _rendererInQuint, _rendererOutQuint,
        _rendererInOutQuint, _rendererInExpo, _rendererOutExpo,
        _rendererInOutExpo, _rendererInCirc, _rendererOutCirc,
        _rendererInOutCirc, _rendererInBack, _rendererOutBack,
        _rendererInOutBack, _rendererInElastic, _rendererOutElastic,
        _rendererInOutElastic, _rendererInBounce, _rendererOutBounce,
        _rendererInOutBounce
    };

    double _res = 0;
    double res = 0;

    if (animation->current_keyframe < animation->count) {
#if false
        TraceLog(LOG_INFO, "[%d] processing keyframe %d", animation->anim_id, animation->current_keyframe);
#endif

        double (*selected_easing)(double) = easings[selected->easing];

        _res = selected_easing(animation->itime / selected->length);

        if (selected->easing == TOLinear) {
            _res = (double)1 - _res;
        }

        res = animation->starting_value + (selected->ending_value * _res);
    } else {
        res = animation->final_value;
    }

    animation->local_current_value = res;

    if (animation->influenced && animation->linked_animation != NULL) {
        struct renderer_animation *anim = (struct renderer_animation *)animation->linked_animation;

        res += anim->current_value;
    }

    animation->current_value = res;

    // animation->time += animation->delta;
    animation->itime += animation->delta;
}

void _ntRendererResetAnimation(struct renderer_animation* animation) {
    if (!animation || !animation->valid) return;

    animation->completed = 0;
    animation->completed_local = 0;
    animation->current_keyframe = 0;
    animation->starting_value = animation->early_value;
    animation->current_value = animation->starting_value;
    animation->itime = 0;
    animation->time = 0;

    if (animation->linked_animation != NULL) {
        _ntRendererResetAnimation(animation->linked_animation);
    }
}

// unloads animation created by _ntRendererLoadAnimation
void _ntRendererUnloadAnimation(struct renderer_animation *animation) {
    if (!animation) return;

    if (animation->keyframes) free(animation->keyframes);
    animation->keyframes = NULL;
    animation->count = 0;
    animation->current_keyframe = 0;

    free(animation);
}
