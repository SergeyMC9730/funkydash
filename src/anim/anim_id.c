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

#include "renderer_animation.h"
#include <stddef.h>

// check if specific animation id exists inside the main node
unsigned char _ntRendererAnimIdExists(struct renderer_animation *animation, int anim_id) {
    return _ntRendererGetEmbeddedAnimation(animation, anim_id) != NULL;
}

double _ntRendererGetAnimationResult(struct renderer_animation *animation, int anim_id) {
    struct renderer_animation *anim = _ntRendererGetEmbeddedAnimation(animation, anim_id);

    if (!anim) return 0;

    return anim->current_value;
}

struct renderer_animation *_ntRendererGetEmbeddedAnimation(struct renderer_animation *animation, int anim_id) {
    if (!animation) return NULL;

    if (animation->anim_id == anim_id) {
        return animation;
    }

    return _ntRendererGetEmbeddedAnimation(animation->linked_animation, anim_id);
}
